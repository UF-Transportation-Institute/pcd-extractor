//
// Created by lightning on 12/30/20.
//

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

class ObjectPCDExtractor {
private:
    /*** Publishers and Subscribers ***/
    ros::NodeHandle node_handle_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher pose_array_pub_;
    ros::Publisher marker_array_pub_;

    bool print_fps_;
    std::string frame_id_;
    int cluster_size_min_;
    int cluster_size_max_;

public:
    ObjectPCDExtractor();

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);
};

ObjectPCDExtractor::ObjectPCDExtractor() {
    point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("/os1_cloud_node/points", 1, &ObjectPCDExtractor::pointCloudCallback, this);

    ros::NodeHandle private_nh("~");

    /*** Parameters ***/
    private_nh.param<bool>("print_fps", print_fps_, true);
    private_nh.param<std::string>("frame_id", frame_id_, "os1_lidar");
}

int frames; clock_t start_time; bool reset = true;//fps
void ObjectPCDExtractor::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2) {
    if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_pc2, *pcl_pc);

    char path[300];
    //dump lable and the cloud
    sprintf(path, "/media/lightning/Samsung_T3/Lidar/clusters/cluster%d.pcd", pcl_pc->header.seq);
    pcl::io::savePCDFile(path, *pcl_pc);

    if(print_fps_)if(++frames>10){std::cerr<<"[object3d_detector]: fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<std::endl;reset = true;}//fps
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_pcd_extractor");
    ObjectPCDExtractor d;
    ros::spin();
    return 0;
}

