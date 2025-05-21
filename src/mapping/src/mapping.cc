#include <mapping/mapping.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace mapping {

void OccGridMap::updateMap(
  const Eigen::Vector3d& sensor_p,
  const std::vector<Eigen::Vector3d>& pc)
{
  // unchanged
  // ... existing code ...
}

void OccGridMap::occ2pc(sensor_msgs::msg::PointCloud2& msg) {
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (infocc.atId(idx) == 1) {
          pcl::PointXYZ pt;
          pt.x = (idx.x() + 0.5) * resolution;
          pt.y = (idx.y() + 0.5) * resolution;
          pt.z = (idx.z() + 0.5) * resolution;
          pcd.push_back(pt);
        }
      }
    }
  }
  msg.header.frame_id = "world";
  // convert PCL to ROS2 message
  pcl::toROSMsg(pcd, msg);
}

void OccGridMap::occ2pc(
  sensor_msgs::msg::PointCloud2& msg,
  double floor,
  double ceil)
{
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (infocc.atId(idx) == 1) {
          pcl::PointXYZ pt;
          pt.x = (idx.x() + 0.5) * resolution;
          pt.y = (idx.y() + 0.5) * resolution;
          pt.z = (idx.z() + 0.5) * resolution;
          if (pt.z > floor && pt.z < ceil) {
            pcd.push_back(pt);
          }
        }
      }
    }
  }
  msg.header.frame_id = "world";
  pcl::toROSMsg(pcd, msg);
}

// inflate methods unchanged...

}  // namespace mapping
