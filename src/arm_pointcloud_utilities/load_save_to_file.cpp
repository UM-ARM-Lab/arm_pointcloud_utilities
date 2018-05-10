#include "arm_pointcloud_utilities/load_save_to_file.h"

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <arc_utilities/serialization_eigen.hpp>
#include <arc_utilities/filesystem.hpp>
#include <arc_utilities/zlib_helpers.hpp>

namespace arm_pointcloud_utilities
{
    uint64_t SerializePointSet(
            const std::string& frame,
            const Eigen::Matrix3Xf& points,
            std::vector<uint8_t>& buffer)
    {
        // Serialize the frame
        const size_t starting_size = buffer.size();
        arc_utilities::SerializeString(frame, buffer);

        // Serialize the points
        arc_utilities::SerializeEigen(points, buffer);

        // Determine how many bytes were written
        const size_t bytes_written = buffer.size() - starting_size;
        return bytes_written;
    }

    std::pair<std::pair<std::string, Eigen::Matrix3Xf>, uint64_t> DeserializePointSet(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        size_t current_position = current;

        // Read in the frame
        const auto frame_deserialized = arc_utilities::DeserializeString<char>(buffer, current_position);
        const std::string& frame = frame_deserialized.first;
        current_position += frame_deserialized.second;

        // Read in the points
        const auto points_deserialized = arc_utilities::DeserializeEigen<Eigen::Matrix3Xf>(buffer, current_position);
        const Eigen::Matrix3Xf& points = points_deserialized.first;
        current_position += points_deserialized.second;

        // Assemble and return the results
        const size_t bytes_read = current_position - current;
        return {{frame, points}, bytes_read};
    }

    void SavePointsetToFile(
            const std::string& frame,
            const Eigen::Matrix3Xf& points,
            const std::string& filepath)
    {
        try
        {
            ROS_DEBUG_NAMED("serialization", "Serializing data");
            std::vector<uint8_t> buffer;
            SerializePointSet(frame, points, buffer);

            // Compress and save to file
            ROS_DEBUG_NAMED("serialization", "Compressing and writing data");
            arc_utilities::CreateDirectory(boost::filesystem::path(filepath).parent_path());
            ZlibHelpers::CompressAndWriteToFile(buffer, filepath);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM_NAMED("serialization", "Saving point set data to file failed: " << e.what());
        }
    }

    std::pair<std::string, Eigen::Matrix3Xf> LoadPointsetFromFile(
            const std::string& filepath)
    {
        ROS_DEBUG_STREAM_NAMED("serialization", "Reading contents of file " << filepath << " and decompressing");
        const std::vector<uint8_t> decompressed = ZlibHelpers::LoadFromFileAndDecompress(filepath);

        ROS_DEBUG_NAMED("serialization", "Deserializing data");
        const auto deserialized_results = DeserializePointSet(decompressed, 0);
        const auto deserialized_bytes_read = deserialized_results.second;
        if (deserialized_bytes_read != decompressed.size())
        {
            throw_arc_exception(std::runtime_error, "deserialization error, read bytes does not match expected bytes");
        }
        return deserialized_results.first;
    }

    void SavePointCloudToFileEigenFormat(
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
            const std::string& filepath)
    {
        const std::string frame_id = cloud->header.frame_id;
        const Eigen::Matrix3Xf points = cloud->getMatrixXfMap(3, 4, 0).transpose();
        SavePointsetToFile(frame_id, points, filepath);
    }
}
