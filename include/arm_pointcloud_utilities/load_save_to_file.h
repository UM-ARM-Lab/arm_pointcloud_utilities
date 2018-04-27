#ifndef ARM_POINTCLOUD_UTILITIES_H
#define ARM_POINTCLOUD_UTILITIES_H

#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>

namespace arm_pointcloud_utilities
{
    uint64_t SerializePointSet(
            const std::string& frame,
            const Eigen::Matrix3Xf& points,
            std::vector<uint8_t>& buffer);

    std::pair<std::pair<std::string, Eigen::Matrix3Xf>, uint64_t> DeserializePointSet(
            const std::vector<uint8_t>& buffer,
            const uint64_t current);


    void SavePointsetToFile(
            const std::string& frame,
            const Eigen::Matrix3Xf& points,
            const std::string& filepath);

    std::pair<std::string, Eigen::Matrix3Xf> LoadPointsetFromFile(
            const std::string& filepath);

    void SavePointCloudToFileEigenFormat(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
            const std::string& filepath);
}

#endif // ARM_POINTCLOUD_UTILITIES_H
