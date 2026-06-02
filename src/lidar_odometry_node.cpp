#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
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
using ApproxSync  = message_filters::sync_policies::ApproximateTime<PointCloud2, Odometry>;

class LidarOdometryNode : public rclcpp::Node
{
public:
  LidarOdometryNode()
  : Node("lidar_odometry_node"),
    icp_odom_(IcpConfig{}),
    tf_broadcaster_(this)
  {
    // BEST_EFFORT QoS — bag 녹화 설정과 무관하게 수신 가능
    auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(500)).best_effort();
    auto imu_qos         = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort();

    // 245Hz SDK 버퍼 구독
    sdk_buffer_sub_ = create_subscription<Odometry>(
      "/state_SDK", best_effort_qos,
      [this](const Odometry::ConstSharedPtr & msg) {
        sdk_buffer_.insert(
          rclcpp::Time(msg->header.stamp),
          SdkPoseBuffer::odomToMatrix(msg));
      });

    // 497Hz IMU 버퍼 구독
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

    // ICP initial guess용 동기화 구독 (BEST_EFFORT)
    points_sub_.subscribe(this, "/points_raw", rmw_qos_profile_sensor_data);
    state_sub_.subscribe(this, "/state_SDK",   rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSync>>(
      ApproxSync(10), points_sub_, state_sub_);
    sync_->registerCallback(
      std::bind(&LidarOdometryNode::syncCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    odom_pub_      = create_publisher<Odometry>("/lidar_odom", 10);
    path_pub_      = create_publisher<Path>("/lidar_path", 10);
    raw_pub_       = create_publisher<PointCloud2>("/points_raw_viz", 10);
    deskewed_pub_  = create_publisher<PointCloud2>("/points_deskewed", 10);

    path_msg_.header.frame_id = "sdk_odom";

    RCLCPP_INFO(get_logger(), "LidarOdometryNode started");
  }

private:
  void syncCallback(
    const PointCloud2::ConstSharedPtr & cloud_msg,
    const Odometry::ConstSharedPtr & odom_msg)
  {
    const double dt_sec =
      std::abs(rclcpp::Time(cloud_msg->header.stamp).seconds() -
               rclcpp::Time(odom_msg->header.stamp).seconds());

    // Gate 1: sync dt가 100ms 초과 → 잘못 매칭된 pair, 스킵
    if (dt_sec > 0.1) {
      RCLCPP_WARN(get_logger(), "[sync] dt=%.1f ms too large, skipping frame", dt_sec * 1000.0);
      return;
    }
    RCLCPP_INFO(get_logger(), "[sync] dt=%.1f ms", dt_sec * 1000.0);

    // SDK 현재 포즈 갱신
    const Eigen::Matrix4f sdk_current = SdkPoseBuffer::odomToMatrix(odom_msg);
    const Eigen::Matrix4f sdk_delta   = sdk_prev_pose_.inverse() * sdk_current;

    // 1. PointCloud2 → CloudIRT
    CloudIRT::Ptr raw_cloud(new CloudIRT);
    pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    // 2. Deskewing (IMU 우선, 없으면 SDK fallback)
    size_t corrected = 0;
    CloudXYZ::Ptr deskewed;
    if (imu_buffer_.snapshot().size() > 2) {
      deskewed = deskewCloudImu(
        raw_cloud, rclcpp::Time(cloud_msg->header.stamp), imu_buffer_, get_logger(), corrected);
    } else {
      deskewed = deskewCloud(
        raw_cloud, rclcpp::Time(cloud_msg->header.stamp), sdk_buffer_, get_logger(), corrected);
    }

    // 보정된 포인트 클라우드 퍼블리시 (frame_id = base_link)
    PointCloud2 deskewed_msg;
    pcl::toROSMsg(*deskewed, deskewed_msg);
    deskewed_msg.header.stamp    = cloud_msg->header.stamp;
    deskewed_msg.header.frame_id = "base_link";
    deskewed_pub_->publish(deskewed_msg);

    // 원본도 base_link frame으로 재퍼블리시
    PointCloud2 raw_viz_msg = *cloud_msg;
    raw_viz_msg.header.frame_id = "base_link";
    raw_pub_->publish(raw_viz_msg);

    // Gate 2: deskewing이 전혀 안 된 경우 → 버퍼 범위 밖, 스킵
    if (corrected == 0 && !raw_cloud->empty()) {
      RCLCPP_WARN(get_logger(), "[deskew] no points corrected, skipping frame");
      sdk_prev_pose_ = sdk_current;
      return;
    }

    // 3. Ground removal → ICP 업데이트
    const CloudXYZ::Ptr no_ground = removeGround(deskewed, get_logger());
    const Eigen::Matrix4f predicted_pose = icp_odom_.currentPose() * sdk_delta;
    const bool converged = icp_odom_.update(no_ground, predicted_pose, get_logger());

    // 4. 수렴 시 /lidar_odom 퍼블리시
    if (converged) {
      const Eigen::Matrix4f & pose = icp_odom_.currentPose();
      Eigen::Quaternionf q_acc(Eigen::Matrix3f(pose.block<3, 3>(0, 0)));
      q_acc.normalize();

      Odometry out;
      out.header.stamp    = cloud_msg->header.stamp;
      out.header.frame_id = "sdk_odom";
      out.child_frame_id  = "base_link";
      out.pose.pose.position.x    = static_cast<double>(pose(0, 3));
      out.pose.pose.position.y    = static_cast<double>(pose(1, 3));
      out.pose.pose.position.z    = static_cast<double>(pose(2, 3));
      out.pose.pose.orientation.x = static_cast<double>(q_acc.x());
      out.pose.pose.orientation.y = static_cast<double>(q_acc.y());
      out.pose.pose.orientation.z = static_cast<double>(q_acc.z());
      out.pose.pose.orientation.w = static_cast<double>(q_acc.w());
      odom_pub_->publish(out);

      // Path 누적 및 퍼블리시
      geometry_msgs::msg::PoseStamped ps;
      ps.header.stamp    = cloud_msg->header.stamp;
      ps.header.frame_id = "sdk_odom";
      ps.pose            = out.pose.pose;
      path_msg_.header.stamp = cloud_msg->header.stamp;
      path_msg_.poses.push_back(ps);
      path_pub_->publish(path_msg_);

      // TF 브로드캐스트: sdk_odom → base_link
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp    = cloud_msg->header.stamp;
      tf_msg.header.frame_id = "sdk_odom";
      tf_msg.child_frame_id  = "base_link";
      tf_msg.transform.translation.x = static_cast<double>(pose(0, 3));
      tf_msg.transform.translation.y = static_cast<double>(pose(1, 3));
      tf_msg.transform.translation.z = static_cast<double>(pose(2, 3));
      tf_msg.transform.rotation.x = static_cast<double>(q_acc.x());
      tf_msg.transform.rotation.y = static_cast<double>(q_acc.y());
      tf_msg.transform.rotation.z = static_cast<double>(q_acc.z());
      tf_msg.transform.rotation.w = static_cast<double>(q_acc.w());
      tf_broadcaster_.sendTransform(tf_msg);

      RCLCPP_INFO(get_logger(), "[pose] x=%.3f  y=%.3f  z=%.3f",
        pose(0, 3), pose(1, 3), pose(2, 3));
    }

    sdk_prev_pose_ = sdk_current;
  }

  // ROS2 인터페이스
  message_filters::Subscriber<PointCloud2>               points_sub_;
  message_filters::Subscriber<Odometry>                  state_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSync>> sync_;
  rclcpp::Subscription<Odometry>::SharedPtr              sdk_buffer_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr                 odom_pub_;
  rclcpp::Publisher<Path>::SharedPtr                     path_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr              raw_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr              deskewed_pub_;

  // 모듈
  SdkPoseBuffer              sdk_buffer_;
  ImuBuffer                  imu_buffer_;
  IcpOdometry                icp_odom_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  Path            path_msg_;
  Eigen::Matrix4f sdk_prev_pose_ = Eigen::Matrix4f::Identity();
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
