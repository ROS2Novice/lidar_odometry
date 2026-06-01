#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/register_point_struct.h>

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float    intensity;
  uint16_t ring;
  float    time;   // 스캔 시작 기준 상대 시간 (초)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
  (float,    x,         x)
  (float,    y,         y)
  (float,    z,         z)
  (float,    intensity, intensity)
  (uint16_t, ring,      ring)
  (float,    time,      time))

using CloudIRT = pcl::PointCloud<PointXYZIRT>;
using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
