#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

#include <custom_msgs/TrackingResultArray.h>
#include <custom_msgs/TrackingResult.h>
#include <custom_msgs/Detection3DArray.h>
#include <custom_msgs/Detection3D.h>
#include <custom_msgs/ObjectHypothesis.h>
#include <custom_msgs/BoundingBox3D.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include "extrinsic_config.hpp"

class TrackerWithCloudNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber lidar_sub_;
    ros::Subscriber tracking_sub_;

    ros::Publisher detection3d_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher marker_pub_;

    std::string lidar_topic_;
    std::string tracking_topic_;
    std::string tracking3d_topic_;
    std::string marker_topic_;
    std::string colored_cloud_topic_;

    float voxel_leaf_size_;
    float cluster_tolerance_;
    int   min_cluster_size_;
    int   max_cluster_size_;

    bool   remove_ground_;
    double ground_dist_thresh_;
    double ground_max_tilt_deg_;
    int    ground_max_iterations_;
    double ground_probability_;
    int    ground_min_inliers_;

    bool   remove_outliers_;
    int    sor_mean_k_;
    double sor_stddev_mul_;

    int dbscan_min_pts_;
    double dbscan_eps_;

    ExtrinsicConfig ex_;

    custom_msgs::TrackingResultArray last_tracks_;
    bool has_tracks_ = false;

public:
    TrackerWithCloudNode();

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void trackingCallback(const custom_msgs::TrackingResultArray::ConstPtr& msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGroundRANSAC(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> densityClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    bool isPointInside2DBbox(const Eigen::Vector3f& p_cam,
                             const custom_msgs::TrackingResult& track);

    void fill3DBbox(custom_msgs::Detection3D& det,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                    int track_id,
                    float score);

    visualization_msgs::MarkerArray createMarkerArray(
        const custom_msgs::Detection3DArray& dets, double lifetime);
};

