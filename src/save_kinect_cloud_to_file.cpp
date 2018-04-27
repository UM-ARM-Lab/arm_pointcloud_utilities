#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/ros_helpers.hpp>

#include "arm_pointcloud_utilities/load_save_to_file.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
using namespace arm_pointcloud_utilities;

class PointCloudSaver
{
public:
    PointCloudSaver()
        : nh_()
        , ph_("~")
        , tf_buffer_()
        , tf_listener_(tf_buffer_)
        , point_cloud_topic_(ROSHelpers::GetParam<std::string>(ph_, "point_cloud_topic", "/kinect2_victor_head/hd/points"))
        , target_frame_(ROSHelpers::GetParam<std::string>(ph_, "target_frame", "victor_root"))
        , cloud_saved_(false)
    {
        point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 1, &PointCloudSaver::savePointCloudCallback, this);
    }

    void spin()
    {
        ros::Rate r(100.0);
        while (ros::ok() && !cloud_saved_)
        {
            ros::spinOnce();
            ROS_INFO_STREAM_THROTTLE(1.0, "Waiting for data from a publisher on topic " << point_cloud_topic_);
            r.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    const std::string point_cloud_topic_;
    const std::string target_frame_;
    ros::Subscriber point_cloud_sub_;
    bool cloud_saved_;

    void savePointCloudCallback(const PointCloud::ConstPtr& msg)
    {
        ros::Time ros_stamp;
        pcl_conversions::fromPCL(msg->header.stamp, ros_stamp);

        if (tf_buffer_.canTransform(target_frame_, msg->header.frame_id, ros_stamp, ros::Duration(1.0)))
        {
            ROS_INFO("Transforming point cloud into target frame");

            // Convert points into an Eigen type in the world frame
            const geometry_msgs::Transform msg_frame_to_target_frame_tf_ros =
                    tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, ros_stamp).transform;
            const Eigen::Isometry3d msg_frame_to_target_frame_tf_eigen =
                    EigenHelpersConversions::GeometryTransformToEigenIsometry3d(msg_frame_to_target_frame_tf_ros);
            const Eigen::Isometry3f transform_as_float = msg_frame_to_target_frame_tf_eigen.cast<float>();

            // Transform the points into the table frame
            const size_t num_points = msg->size();
            Eigen::Matrix3Xf points(3, num_points);
            for (size_t idx = 0; idx < num_points; ++idx)
            {
                const auto& pcl_point = msg->points[idx];
                points.col(idx) = transform_as_float * Eigen::Vector3f(pcl_point.x, pcl_point.y, pcl_point.z);
            }

            // Load the parameters that set where we are saving the file
            const std::string filename_prefix = ROSHelpers::GetParam<std::string>(ph_, "file_name_prefix", "./logs/point_cloud");
            const std::string filename_suffix = arc_helpers::GetCurrentTimeAsString();
            const std::string filename = filename_prefix + "__" + filename_suffix + ".compressed";

            ROS_INFO_STREAM("Serializing, compressing, and writting to file: " << filename);
            SavePointsetToFile(target_frame_, points, filename);

            ROS_INFO_STREAM("Testing loading, decompressing, and deserializing: " << filename);
            const std::pair<std::string, Eigen::Matrix3Xf> deserialized = LoadPointsetFromFile(filename);
            assert(deserialized.first == target_frame_);

            const Eigen::Matrix3Xf& points_recovered = deserialized.second;
            assert(points_recovered.cols() == points.cols());
            for (ssize_t col = 0; col < points.cols(); ++col)
            {
                for (ssize_t row = 0; row < 3; ++row)
                {
                    if (std::isnan(points(row, col)))
                    {
                        assert(std::isnan(points_recovered(row, col)));
                    }
                    else
                    {
                        assert(points(row, col) == points_recovered(row, col));
                    }
                }
            }

            cloud_saved_ = true;
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE(1.0, "Cannot convert between message frame: "
                                     << msg->header.frame_id << " and target frame " << target_frame_);
        }
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "save_kinect_cloud_to_file_node");

    PointCloudSaver saver;
    saver.spin();

    return EXIT_SUCCESS;
}
