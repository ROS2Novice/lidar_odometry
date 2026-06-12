#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_odometry/point_types.hpp"
#include "lidar_odometry/sdk_pose_buffer.hpp"
#include "lidar_odometry/imu_buffer.hpp"
#include "lidar_odometry/deskew.hpp"
#include "lidar_odometry/ground_removal.hpp"
#include "lidar_odometry/icp_odometry.hpp"

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Odometry    = nav_msgs::msg::Odometry;
using Path        = nav_msgs::msg::Path;

class LidarOdometryNode : public rclcpp::Node
{
public:
  LidarOdometryNode()
  : Node("lidar_odometry_node"),
    icp_odom_(IcpConfig{}),
    tf_broadcaster_(this)
  {
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(500)).best_effort();
    auto imu_qos         = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort();
    auto cloud_qos       = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // SDK 버퍼 구독 + path 발행 (원점 기준 정규화)
    sdk_buffer_sub_ = create_subscription<Odometry>(
      "/state_SDK", best_effort_qos,
      [this](const Odometry::ConstSharedPtr & msg) {
        const Eigen::Matrix4f T = SdkPoseBuffer::odomToMatrix(msg);
        sdk_buffer_.insert(rclcpp::Time(msg->header.stamp), T);

        if (!sdk_origin_set_) {
          sdk_origin_     = T;
          sdk_origin_set_ = true;
        }

        const Eigen::Matrix4f rel = sdk_origin_.inverse() * T;
        Eigen::Quaternionf q(Eigen::Matrix3f(rel.block<3, 3>(0, 0)));
        q.normalize();

        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp    = msg->header.stamp;
        ps.header.frame_id = "sdk_odom";
        ps.pose.position.x    = static_cast<double>(rel(0, 3));
        ps.pose.position.y    = static_cast<double>(rel(1, 3));
        ps.pose.position.z    = static_cast<double>(rel(2, 3));
        ps.pose.orientation.x = static_cast<double>(q.x());
        ps.pose.orientation.y = static_cast<double>(q.y());
        ps.pose.orientation.z = static_cast<double>(q.z());
        ps.pose.orientation.w = static_cast<double>(q.w());
        sdk_path_msg_.header.stamp    = msg->header.stamp;
        sdk_path_msg_.header.frame_id = "sdk_odom";
        sdk_path_msg_.poses.push_back(ps);
        sdk_path_pub_->publish(sdk_path_msg_);
      });

    // IMU 버퍼 구독
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu_raw", imu_qos,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr & msg) {
        const auto & q = msg->orientation;
        imu_buffer_.insert(
          rclcpp::Time(msg->header.stamp),
          Eigen::Quaternionf(
            static_cast<float>(q.w), static_cast<float>(q.x),
            static_cast<float>(q.y), static_cast<float>(q.z)));
      });

    // LiDAR 직접 구독 — SDK 버퍼에서 타임스탬프 보간으로 포즈 조회
    cloud_sub_ = create_subscription<PointCloud2>(
      "/points_raw", cloud_qos,
      std::bind(&LidarOdometryNode::cloudCallback, this, std::placeholders::_1));

    odom_pub_     = create_publisher<Odometry>("/lidar_odom", 10);
    path_pub_     = create_publisher<Path>("/lidar_path", 10);
    sdk_path_pub_ = create_publisher<Path>("/sdk_path", 10);
    raw_pub_      = create_publisher<PointCloud2>("/points_raw_viz", 10);
    deskewed_pub_ = create_publisher<PointCloud2>("/points_deskewed", 10);

    path_msg_.header.frame_id = "sdk_odom";

    RCLCPP_INFO(get_logger(), "LidarOdometryNode started");
  }

private:
  void cloudCallback(const PointCloud2::ConstSharedPtr & cloud_msg)
  {
    const rclcpp::Time cloud_stamp(cloud_msg->header.stamp);

    // SDK 버퍼에서 클라우드 타임스탬프의 포즈 보간
    auto buf = sdk_buffer_.snapshot();
    Eigen::Matrix4f sdk_current;
    if (!interpolatePose(buf, cloud_stamp, sdk_current)) {
      RCLCPP_WARN(get_logger(), "[cloud] SDK buffer not ready yet");
      return;
    }

    // 첫 프레임: SDK 절대 포즈를 기준으로 저장 → 이후 delta가 원점 기준
    if (first_cloud_) {
      sdk_prev_pose_ = sdk_current;
      first_cloud_   = false;
      return;
    }

    const Eigen::Matrix4f sdk_delta = sdk_prev_pose_.inverse() * sdk_current;

    // 1. PointCloud2 → CloudIRT
    CloudIRT::Ptr raw_cloud(new CloudIRT);
    pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    // 2. Deskewing (IMU 우선, 없으면 SDK fallback)
    size_t corrected = 0;
    CloudXYZ::Ptr deskewed;
    if (imu_buffer_.snapshot().size() > 2) {
      deskewed = deskewCloudImu(
        raw_cloud, cloud_stamp, imu_buffer_, sdk_buffer_, get_logger(), corrected);
    } else {
      deskewed = deskewCloud(
        raw_cloud, cloud_stamp, sdk_buffer_, get_logger(), corrected);
    }

    PointCloud2 deskewed_msg;
    pcl::toROSMsg(*deskewed, deskewed_msg);
    deskewed_msg.header.stamp    = cloud_msg->header.stamp;
    deskewed_msg.header.frame_id = "base_link";
    deskewed_pub_->publish(deskewed_msg);

    PointCloud2 raw_viz_msg = *cloud_msg;
    raw_viz_msg.header.frame_id = "base_link";
    raw_pub_->publish(raw_viz_msg);

    if (corrected == 0 && !raw_cloud->empty()) {
      RCLCPP_WARN(get_logger(), "[deskew] no points corrected, skipping");
      sdk_prev_pose_ = sdk_current;
      return;
    }

    // 3. Ground removal → ICP
    const GroundResult gr           = removeGround(deskewed, get_logger());
    const Eigen::Matrix4f predicted = icp_odom_.currentPose() * sdk_delta;
    const bool converged            = icp_odom_.update(gr.no_ground, predicted, get_logger());

    // 4. 수렴 시 오도메트리 발행
    if (converged) {
      const Eigen::Matrix4f & pose = icp_odom_.currentPose();
      Eigen::Quaternionf q(Eigen::Matrix3f(pose.block<3, 3>(0, 0)));
      q.normalize();

      // Ground plane 기반 z 보정
      //   n_s.z > 0.95 (평지)      : EMA smoothed sensor_height → 안정적 z
      //   n_s.z 0.80~0.95 (완만한 경사): raw sensor_height delta → 높이 추적
      //   그 외                    : ICP z 그대로
      float publish_z = pose(2, 3);
      if (gr.plane_valid) {
        Eigen::Vector3f n_s(gr.a, gr.b, gr.c);
        const float n_norm = n_s.norm();
        if (n_norm > 1e-6f) {
          n_s /= n_norm;
          if (n_s.z() < 0.0f) n_s = -n_s;
          const float sensor_h = std::abs(gr.d) / n_norm;  // 센서~지면 수직 거리

          if (n_s.z() > 0.95f) {                           // 평지
            if (first_ground_frame_) {
              init_z_          = pose(2, 3);
              init_sensor_h_   = sensor_h;
              smooth_sensor_h_ = sensor_h;
              first_ground_frame_ = false;
            } else {
              constexpr float kAlpha = 0.05f;
              smooth_sensor_h_ = kAlpha * sensor_h + (1.0f - kAlpha) * smooth_sensor_h_;
            }
            publish_z = init_z_ + (init_sensor_h_ - smooth_sensor_h_);
          } else if (n_s.z() > 0.80f && !first_ground_frame_) {  // 완만한 경사
            publish_z = init_z_ + (init_sensor_h_ - sensor_h);
          }
        }
      }

      Odometry out;
      out.header.stamp    = cloud_msg->header.stamp;
      out.header.frame_id = "sdk_odom";
      out.child_frame_id  = "base_link";
      out.pose.pose.position.x    = static_cast<double>(pose(0, 3));
      out.pose.pose.position.y    = static_cast<double>(pose(1, 3));
      out.pose.pose.position.z    = static_cast<double>(publish_z);
      out.pose.pose.orientation.x = static_cast<double>(q.x());
      out.pose.pose.orientation.y = static_cast<double>(q.y());
      out.pose.pose.orientation.z = static_cast<double>(q.z());
      out.pose.pose.orientation.w = static_cast<double>(q.w());
      odom_pub_->publish(out);

      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp    = cloud_msg->header.stamp;
      ps.header.frame_id = "sdk_odom";
      ps.pose            = out.pose.pose;
      path_msg_.header.stamp = cloud_msg->header.stamp;
      path_msg_.poses.push_back(ps);
      path_pub_->publish(path_msg_);

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp    = cloud_msg->header.stamp;
      tf_msg.header.frame_id = "sdk_odom";
      tf_msg.child_frame_id  = "base_link";
      tf_msg.transform.translation.x = static_cast<double>(pose(0, 3));
      tf_msg.transform.translation.y = static_cast<double>(pose(1, 3));
      tf_msg.transform.translation.z = static_cast<double>(publish_z);
      tf_msg.transform.rotation.x = static_cast<double>(q.x());
      tf_msg.transform.rotation.y = static_cast<double>(q.y());
      tf_msg.transform.rotation.z = static_cast<double>(q.z());
      tf_msg.transform.rotation.w = static_cast<double>(q.w());
      tf_broadcaster_.sendTransform(tf_msg);

      const float sdk_z_rel = (sdk_origin_.inverse() * sdk_current)(2, 3);
      RCLCPP_INFO(get_logger(), "[pose] x=%.3f  y=%.3f  z=%.3f  (icp_z=%.3f  sh=%.3f  sdk_z=%.3f)",
        pose(0, 3), pose(1, 3), publish_z, pose(2, 3), smooth_sensor_h_, sdk_z_rel);
    }

    sdk_prev_pose_ = sdk_current;
  }

  rclcpp::Subscription<PointCloud2>::SharedPtr               cloud_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr                  sdk_buffer_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr     imu_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr                     odom_pub_;
  rclcpp::Publisher<Path>::SharedPtr                         path_pub_;
  rclcpp::Publisher<Path>::SharedPtr                         sdk_path_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr                  raw_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr                  deskewed_pub_;

  SdkPoseBuffer                 sdk_buffer_;
  ImuBuffer                     imu_buffer_;
  IcpOdometry                   icp_odom_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  Path            path_msg_;
  Path            sdk_path_msg_;
  Eigen::Matrix4f sdk_prev_pose_  = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f sdk_origin_     = Eigen::Matrix4f::Identity();
  bool            sdk_origin_set_ = false;
  bool            first_cloud_    = true;
  bool            first_ground_frame_ = true;
  float           init_z_         = 0.0f;
  float           init_sensor_h_  = 0.0f;
  float           smooth_sensor_h_= 0.0f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
