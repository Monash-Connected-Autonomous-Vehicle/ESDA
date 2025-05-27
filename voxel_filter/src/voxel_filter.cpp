#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <string>

using std::placeholders::_1;

class VoxelFilterNode: public rclcpp::Node
{
    public:
        VoxelFilterNode() : Node("voxel_filter_node")
        {
            subscribe_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                // this might be different when using the VLP32
                "/velodyne_points", rclcpp::SensorDataQoS(),
                std::bind(&VoxelFilterNode::pointCloudCallback, this, _1));

            // published to /velodyne_voxel_filtered ROS2 topic
            publish_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_voxel_filtered", 10);
            
            RCLCPP_INFO(this->get_logger(), "Voxel filter node started.");
        }
    
    private:
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // Create new PCL PC2 and convert Velodyne data into PCL format
            pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2());
            pcl_conversions::toPCL(*msg, *pcl_pc2);

            // Filtered PC2 
            pcl::PCLPointCloud2::Ptr voxel_filtered (new pcl::PCLPointCloud2());

            // (Debug) PC points before filter
            std::cout<<"Source Cloud Points "<< pcl_pc2->width * pcl_pc2->height<< std::endl;

            // Create filtering object
            pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
            voxel_filter.setInputCloud(pcl_pc2);
            voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // leaf size (adjust as needed for desired downsampling, )
            voxel_filter.filter(*voxel_filtered);

            // (Debug) PC points after filter
            std::cout<<"Voxel Cloud Points "<< voxel_filtered->width * voxel_filtered->height<< std::endl;

            // Convert back to ROS2/PointCloud2 sensor msg
            sensor_msgs::msg::PointCloud2 output;
            pcl_conversions::fromPCL(*voxel_filtered, output);
            output.header = msg->header; // idk what this does


            publish_->publish(output);
        }
        
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscribe_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publish_;
};

// main function
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelFilterNode>());
    rclcpp::shutdown();
    return 0;
}