#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include<geometry_msgs/PoseStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.h>

#include <boost/make_shared.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub_cloudmap;

nav_msgs::Odometry odom;
Eigen::Isometry3d T_last;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap; 
pcl::visualization::CloudViewer viewer("viewer");
int count;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom.pose = msg->pose;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    pcl::PointXYZRGB P;
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_pt( new pcl::PointCloud<pcl::PointXYZRGB>() );
    pcl::fromROSMsg(*cloud,point_cloud);
    int size = cloud->height * cloud->width;
    Eigen::Quaterniond q(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,odom.pose.pose.orientation.z);
    Eigen::Isometry3d T(q);
    Eigen::Vector3d t(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
    T.pretranslate(t);
    Eigen::Isometry3d Tc = T * T_last.inverse();
    Eigen::Matrix3d Rc;
    Eigen::Vector3d tc;
            Rc << Tc(0,0),Tc(0,1),Tc(0,2),
                  Tc(1,0),Tc(1,1),Tc(1,2),
                  Tc(2,0),Tc(2,1),Tc(2,2);
            tc << Tc(0,3),Tc(1,3),Tc(2,3);
    Sophus::SE3 d_T(Rc,tc);
    Eigen::Matrix<double,6,1> d = d_T.log();

    for (int i = 0; i < size; i += 5) 
    {
        if(point_cloud.points[i].x ==0 || point_cloud.points[i].x > 10) continue;
        Eigen::Vector3d p(point_cloud.points[i].x,point_cloud.points[i].y,
                            point_cloud.points[i].z);
        Eigen::Vector3d p_world = T*p;
        point_cloud.points[i].x = p_world[0];
        point_cloud.points[i].y = p_world[1];
        point_cloud.points[i].z = p_world[2];
        point_cloud_pt->points.push_back(point_cloud[i]);
    }
    if(d.norm() > 0.1)
    {
        //移除点云测量噪声点（离群点）
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp( new pcl::PointCloud<pcl::PointXYZRGB> );
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(point_cloud_pt);
        statistical_filter.filter( *tmp );
        (*globalMap) += *tmp;

        T_last = T;
        count++;
    }
    // voxel filter 体素化网格减少点数量
   /* pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsp( new pcl::PointCloud<pcl::PointXYZRGB>() );
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter; 
    voxel_filter.setLeafSize( 0.0002, 0.0002, 0.0002 );       // 分辨率
    voxel_filter.setInputCloud( globalMap );
    voxel_filter.filter( *tsp );
    globalMap->swap( *tsp );*/
    viewer.showCloud( globalMap );
    ROS_INFO_STREAM("d.norm:" <<d.norm());
    ROS_INFO_STREAM("pointcloud:" <<globalMap->size());
    sensor_msgs::PointCloud2 gmap;
    pcl::toROSMsg(*globalMap, gmap);
    gmap.header.frame_id = "map";
    gmap.is_dense = false;
    pub_cloudmap.publish(gmap);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_cloudmap");
    ros::NodeHandle n;
    ros::Subscriber zed_odom = n.subscribe<nav_msgs::Odometry> ("zed/odom",10,odom_cb);
    ros::Subscriber zed_cloud = n.subscribe<sensor_msgs::PointCloud2> 
                                ("zed/point_cloud/cloud_registered",10,cloud_cb);

    pub_cloudmap = n.advertise<sensor_msgs::PointCloud2> ("point_cloud_map", 30);
    ros::Rate r(60);

    T_last = Eigen::Isometry3d::Identity();
    globalMap = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGB> >( );
    globalMap->is_dense = false;
    count = 0;

    while(n.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}