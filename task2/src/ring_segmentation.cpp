#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "std_msgs/ColorRGBA.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "task2/ColorAndPose.h"

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;
ros::Publisher ring_pose_publisher;
ros::Publisher debug;
ros::Publisher debug_string;

#include <sstream>

namespace gdb {
	template <typename T> 
	void assemble(std::stringstream& ss, const T& t) {
	    ss << t;
	}
	
	template <typename First, typename... Rest>
	void assemble(std::stringstream& ss, const First& first, const Rest&... rest) {
	    ss << first << " ";
	    assemble(ss, rest...); // recursive call using pack expansion syntax
	}
	
	template<typename... Args>
	std::string format(Args... args) {
		std::stringstream ss;
		assemble(ss, args...);
		return ss.str();
	}

    template<typename... Args>
    void publish(Args... args) {
        std_msgs::String msg;
        msg.data = format(args...);
        debug_string.publish(msg);
    }
};

tf2_ros::Buffer tf2_buffer;

double min_z, max_z, min_y, max_y;

typedef pcl::PointXYZRGB PointT;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
    // All the objects needed

    ros::Time time_rec, time_test;
    time_rec = ros::Time::now();

    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    Eigen::Vector4f centroid;

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>), cloudF(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>), cloud_normalsF(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_ring(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_ring(new pcl::PointIndices);

    // Read in the cloud data
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud_filtered);

    // Estimate point normals
    ROS_INFO("Kaj se dogaja %d, %d", cloud_filtered->width, cloud_filtered->height);

    if (cloud_filtered->points.empty())
        return;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    // int initial_size = cloud_filtered->height * cloud_filtered->width;
    // int count = 0;

    // std::cout << "------------------------" << std::endl;
    // std::cout << "Size before: " << cloud_filtered->height * cloud_filtered->width;
    // std::cout << ", " << cloud_normals->height * cloud_normals->width << std::endl;
    // while (cloud_filtered->height * cloud_filtered->width > initial_size * 0.1 && count++ < 3)
    // {
        // seg.setInputCloud(cloud_filtered);
        // seg.setInputNormals(cloud_normals);
        // // Obtain the plane inliers and coefficients
        // seg.segment(*inliers_plane, *coefficients_plane);

        // // Remove the planar inliers, extract the rest
        // extract.setInputCloud(cloud_filtered);
        // extract.setIndices(inliers_plane);
        // extract.setNegative(true);

        // extract_normals.setInputCloud(cloud_normals);
        // extract_normals.setIndices(inliers_plane);
        // extract_normals.setNegative(true);

        // extract.filter(*cloudF);
        // cloud_filtered.swap(cloudF);

        // extract_normals.filter(*cloud_normalsF);
        // cloud_normals.swap(cloud_normalsF);
    // }

    // std::cout << "Size after: " << cloud_filtered->height * cloud_filtered->width;
    // std::cout << ", " << cloud_normals->height * cloud_normals->width << ", count: " << count << std::endl;

    // if (cloud_filtered->height * cloud_filtered->width < 10)
    //     return;

    pcl::PCLPointCloud2 outcloud_plane;
    // pcl::toPCLPointCloud2(*cloud_filtered, outcloud_plane);
    // pubx.publish(outcloud_plane);

    // Create the segmentation object for ring segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.005);
    seg.setRadiusLimits(0.05, 0.2);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    // Obtain the ring inliers and coefficients
    seg.segment(*inliers_ring, *coefficients_ring);

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_ring);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_ring(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_ring);
    pcl::toPCLPointCloud2(*cloud_ring, outcloud_plane);
    debug.publish(outcloud_plane);
    
    
    

    if (!cloud_ring->points.empty() && (cloud_ring->points.size() * 2.5 > cloud_filtered->points.size()) && cloud_ring->points.size() > 300)
    {
        gdb::publish("Inliers: ", cloud_ring->points.size(), ", Outliers: ", cloud_filtered->points.size());
        float radius_val = coefficients_ring->values[3];
        float radius_target = 0.12;

        uint32_t r = 0;
        uint32_t g = 0;
        uint32_t b = 0;
        uint32_t rgb;
        int size = cloud_ring->points.size();
        int i = 0;
        int increment = 500;
        for (; i < size; i += increment)
        {
            rgb = *reinterpret_cast<int *>(&cloud_ring->points[i].rgb);
            r += (rgb >> 16) & 0x0000ff;
            g += (rgb >> 8) & 0x0000ff;
            b += (rgb)&0x0000ff;
        }

        r = r / (i / increment);
        g = g / (i / increment);
        b = b / (i / increment);

        // printf("r: %d, g: %d, b: %d, samples: %d \n", r, g, b, i / increment);

        // std::cout << "Radius: " << radius_val << ", Size: " << size << std::endl;

        // if (abs(radius_val - radius_target) > 0.005 || size < 2500 || size > 10000)
        //     return;
        gdb::publish("Radius:", radius_val);

        // std::cout << "ring found" << std::endl;
        pcl::compute3DCentroid(*cloud_ring, centroid);

        //Create a point in the "camera_rgb_optical_frame"
        geometry_msgs::PointStamped point_camera;
        geometry_msgs::PointStamped point_map;
        visualization_msgs::Marker marker;
        geometry_msgs::TransformStamped tss;

        point_camera.header.frame_id = "camera_rgb_optical_frame";
        point_camera.header.stamp = ros::Time::now();

        point_map.header.frame_id = "map";
        point_map.header.stamp = ros::Time::now();

        point_camera.point.x = centroid[0];
        point_camera.point.y = centroid[1];
        point_camera.point.z = centroid[2];

        try
        {
            time_test = ros::Time::now();
            tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform warning: %s\n", ex.what());
        }

        tf2::doTransform(point_camera, point_map, tss);

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "ring";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = point_map.point.x;
        marker.pose.position.y = point_map.point.y;
        marker.pose.position.z = point_map.point.z;
        marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
	    marker.scale.y = 0.1;
	    marker.scale.z = 0.1;

        marker.color.r=0.0f;
        marker.color.g=1.0f;
        marker.color.b=0.0f;
        marker.color.a=1.0f;

	    marker.lifetime = ros::Duration();

	    pubm.publish (marker);

        // exercise6:CylinderSegmentation msg;
        task2::ColorAndPose msg;
        std_msgs::ColorRGBA color;

        color.r = r;
        color.g = g;
        color.b = b;
        color.a = 1;

        msg.pose = point_map;
        msg.color = color;

        // ring_pose_publisher.publish(msg);
        
        // printf("%d %d %d\n", r, g, b);

        // pcl::PCLPointCloud2 outcloud_cylinder;
        // pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
        // puby.publish(outcloud_cylinder);
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "ring_segmentation");
    ros::NodeHandle nh;

    std::cout << "Started" << std::endl;

    // For transforming between coordinate frames
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

    nh.param<double>("min_y", min_y, -0.3);
    nh.param<double>("max_y", max_y, 0.09);
    nh.param<double>("min_z", min_z, -0.2);
    nh.param<double>("max_z", max_z, 2.0);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pubx = nh.advertise<pcl::PCLPointCloud2>("planes", 1);
    puby = nh.advertise<pcl::PCLPointCloud2>("ring", 1);
    pubm = nh.advertise<visualization_msgs::Marker>("detected_ring",1);

    ring_pose_publisher = nh.advertise<task2::ColorAndPose>("ring_detected", 1);
    debug = nh.advertise<pcl::PCLPointCloud2>("debug", 1);
    debug_string = nh.advertise<std_msgs::String>("gdb", 1);

    // Spin
    ros::spin();
}