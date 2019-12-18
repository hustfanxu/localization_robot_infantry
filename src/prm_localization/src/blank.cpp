#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//eigen
#include <Eigen/Dense>
//nodelet
//#include <nodelet/nodelet.h>
//#include <pluginlib/class_list_macros.h>
//cpp
#include <ctime>
#include <boost/circular_buffer.hpp>
//user
#include <prm_localization/transform_utility.hpp>
using namespace std;
using namespace Eigen;
using PointT = pcl::PointXYZ;



int main(int argc, char *argv[]) {
    /** tf research **/
    tf::StampedTransform transform1;
    tf::Quaternion tfquaternion;
    tf::Vector3 tfVector3;
//    Eigen::Quaternionf eigenQ = euler2quat(1.5,2.0,-0.5);
    tfquaternion.setEuler(1.5,2.0,-0.5);
    tfVector3.setValue(2,3,4);
    transform1.setOrigin(tfVector3);
    transform1.setRotation(tfquaternion);
    // cout<<transform1.getBasis()<<endl;
    /** eigen research**/
//    Matrix4f m4a ;
//    m4a.setRandom();
//    cout<<m4a(0)<<endl;
//    cout<<m4a(1)<<endl;
//    cout<<m4a(2)<<endl;
//    cout<<m4a(4)<<endl;
//    cout<<m4a(5)<<endl;
//    cout<<m4a(6)<<endl;
//    cout<<m4a(7)<<endl;
//
//cout<<m4a<<endl;
    /**send odom**/
//    ros::init(argc, argv, "send_odom");
//    ros::NodeHandle nh;
//    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",50);
//    ros::Publisher get_pmsg_pub = nh.advertise<nav_msgs::Odometry>("/stamp",5);
//
//    nav_msgs::Odometry odometry;
//    get_pmsg_pub.publish(odometry);
//    sleep(3);
//    odom_pub.publish(odometry);

//    /**hello iterator**/
//    boost::circular_buffer<int> sham;
//    sham.set_capacity(4);
////    nav_msgs::OdometryPtr odom (new nav_msgs::Odometry());
//    size_t  a = sham.size();
//    sham.push_back(1);
//    sham.push_back(2);
//    sham.push_back(3);
//    sham.push_back(4);
//    sham.push_back(5);
//    size_t b = sham.size();
//    for (boost::circular_buffer<int>::const_iterator i = sham.end()-1;i!=sham.begin();i--){
//        cout<<*i<<endl;
//    }

    //    boost::circular_buffer<nav_msgs::OdometryConstPtr> sham;
//    sham.set_capacity(4);
////    nav_msgs::OdometryPtr odom (new nav_msgs::Odometry());
//    size_t  a = sham.size();
//    sham.push_back(odom);
//    sham.push_back(odom);
//    size_t b = sham.size();
//    for (boost::circular_buffer<nav_msgs::OdometryConstPtr>::const_iterator i = sham.end()-1;i!=sham.begin();i--){
//        nav_msgs::Odometry odometry = **i;
////        NODELET_INFO("check a odom_msg");
//        if(odometry.header.stamp <= odometry.header.stamp){
//            Quaternionf q(odometry.pose.pose.orientation.w,odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z);
//            Matrix4f odom_pose;
//            odom_pose.block(0,0,3,3)=quat2rot(q);
//            odom_pose(0,3)=odometry.pose.pose.position.x;
//            odom_pose(1,3)=odometry.pose.pose.position.y;
//            odom_pose(2,3)=odometry.pose.pose.position.z;
//            break;
//        }
//    }

    /**do a rotm**/
    //right
//    Eigen::Quaternionf quaternionfr;
//    Eigen::Matrix4f rotmr;
//    quaternionfr.x()=0.240932;
//    quaternionfr.y()=0.249054;
//    quaternionfr.z()=-0.671951;
//    quaternionfr.w()=0.654527;
//    rotmr.block(0,0,3,3) = quat2rot(quaternionfr);
//    rotmr(0,3)=-0.342517;
//    rotmr(1,3)=-0.759176;
//    rotmr(2,3)=-0.323078;
//    rotmr(3,0)=0;
//    rotmr(3,2)=0;
//    //left
//    Eigen::Quaternionf quaternionfleft;
//    Eigen::Matrix4f rotml;
//    rotml.setIdentity();
//    quaternionfleft.x()=-0.259988;
//    quaternionfleft.y()=0.247313;
//    quaternionfleft.z()=0.617506;
//    quaternionfleft.w()=0.69995;
//    rotml.block(0,0,3,3) = quat2rot( quaternionfleft);
//    rotml(0,3)=-0.321385;
//    rotml(1,3)=0.678507;
//    rotml(2,3)=-0.354922;
//
//    cout<<"rotmr"<<endl<<rotmr<<endl;
//    cout<<"rotml"<<endl<<rotml<<endl;
    /**nvidva pcprocess**/
    // void initializePointCloudProcessing(){
    //     // allocate point cloud buffer to store accumulated points
    //     gAccumulatedPoints.type     = DW_MEMORY_TYPE_CUDA;
    //     gAccumulatedPoints.capacity = gLidarProperties.pointsPerSpin;
    //     dwPointCloud_createBuffer(&gAccumulatedPoints);

    //     // initialize DriveWorks PointCloudAccumulator module which will accumulate individual lidar packets to a point cloud
    //     dwPointCloudAccumulatorParams accumParams{};
    //     accumParams.egomotion                = gEgomotion;
    //     accumParams.memoryType               = DW_MEMORY_TYPE_CUDA;
    //     accumParams.enableMotionCompensation = true;
    //     dwPointCloudAccumulator_initialize(&gAccumulator, &accumParams, &gLidarProperties, gContext);

    //     // allocate point cloud buffer to store stitched points
    //     gStitchedPoints.type     = DW_MEMORY_TYPE_CUDA;
    //     gStitchedPoints.capacity = getNumberOfPointsPerSpinFromAllLidars();
    //     dwPointCloud_createBuffer(&gStitchedPoints);

    //     // initialize DriveWorks PointCloudStitcher which will combine multiple point clouds into one
    //     dwPointCloudStitcher_initialize(&gStitcher, gContext);

    //     // allocate buffer to generate depthmap representation from lidar points
    //     gDepthMap.type     = DW_MEMORY_TYPE_CUDA;
    //     gDepthMap.capacity = getDepthmapWidth() * getDepthmapHeight();
    //     dwPointCloud_createBuffer(&gDepthMap);

    //     // create RangeImageCreator which will generate 2.5D depthmap as well as range image representation from unorganized accumulated points
    //     dwPointCloudRangeImageCreator_initialize(&gRangeImageCreator, &gRangeImageCreatorProperties, gContext);

    //     // initialize DriveWorks ICP module
    //     dwPointCloudICPParams icpParams{};
    //     icpParams.icpType  = DW_POINT_CLOUD_ICP_TYPE_DEPTH_MAP; // optimized ICP for 2.5D depthmap representation of a point cloud
    //     dwPointCloudICP_initialize(&gICP, &icpParams, gContext);
    // }
    /**learn copy_if back insert**/
//        std::string globalmap_pcd = "/home/vickylzy/WorkSPacesROS/catkin_ws/src/prm_localization/data/shunYuFactory.pcd";//private_nh.param<std::string>("globalmap_pcd", "");
//        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
//        pcl::io::loadPCDFile(globalmap_pcd, *cloud);
//
//        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
//        filtered->reserve(cloud->size());
//
//        std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
//                     [&](const PointT& p) {
//                         double d = p.getVector3fMap().norm();
//                         return d > 0.1 && d < 2;
//                     }
//        );
//
//        filtered->width = filtered->size();
//        filtered->height = 1;
//        filtered->is_dense = false;
//
//        filtered->header = cloud->header;



    /**analyse parameter of icp**/
//    std::string globalmap_pcd = "/home/vickylzy/WorkSPacesROS/catkin_ws/src/prm_localization/data/shunYuFactory.pcd";//private_nh.param<std::string>("globalmap_pcd", "");
//    std::string currmap_pcd = "/home/vickylzy/WorkSPacesROS/catkin_ws/src/prm_localization/data/shunYu1.pcd";//private_nh.param<std::string>("globalmap_pcd", "");
//
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//    pcl::io::loadPCDFile(globalmap_pcd, *full_cloud);
//    full_cloud->header.frame_id = "map";
//    pcl::io::loadPCDFile(currmap_pcd, *curr_cloud);
//    curr_cloud->header.frame_id = "map";
//
//    clock_t init = clock();
//
//    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//    icp.setInputTarget(full_cloud);
//    icp.setInputSource(curr_cloud);
//    //para
//    icp.setMaximumIterations(50);
//    icp.setTransformationEpsilon(1e-7);
//    icp.setRANSACOutlierRejectionThreshold(0.04);
//    icp.setMaxCorrespondenceDistance(4);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
//    /** analyse RANSACOutlierRejectionThreshold and MaxCorrespondenceDistance**/
//    for (double i = 0.01; i <0.15 ; i+=0.01) {
//        icp.setRANSACOutlierRejectionThreshold(i);
//        icp.setMaxCorrespondenceDistance(i*100);
//        clock_t start = clock();
//        icp.align(*result);
//        clock_t end = clock();
//        cout<<"registration in "<< (double)(end  - start) / CLOCKS_PER_SEC << "second" << endl;
//        cout<<"with"<< "registration.setRANSACOutlierRejectionThreshold() = "<< i <<endl;
//        cout<<"registration.getFitnessScore() = "<<icp.getFitnessScore()<<endl;
//        cout<<endl;
//
//    }
//    /** analyse TransformationEpsilon**/
//    for (double i = 1e-11; i <=1e-7 ; i*=10) {
//        icp.setTransformationEpsilon(i);
//        clock_t start = clock();
//        icp.align(*result);
//        clock_t end = clock();
//        cout<<"registration in "<< (double)(end  - start) / CLOCKS_PER_SEC << "second" << endl;
//        cout<<"with"<< "TransformationEpsilon() = "<< i <<endl;
//        cout<<"registration.getFitnessScore() = "<<icp.getFitnessScore()<<endl;
//        cout<<endl;
//
//    }

/**pcl_viewer**/
//    clock_t start = clock();

//    registration.align(*result);

//    clock_t end = clock();
    //color
//    for (size_t i = 0; i < full_cloud->points.size() ; ++i) {
//        full_cloud->points[i].r = 255;
//        full_cloud->points[i].g = 0;
//        full_cloud->points[i].b = 0;
//    }
//    for (size_t i = 0; i < result->points.size() ; ++i) {
//        result->points[i].r=255;
//        result->points[i].g=255;
//        result->points[i].b=255;
//    }
//    cout<<"init in "<< (double)(start  - init) / CLOCKS_PER_SEC << "second" << endl;
//
//    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show (new pcl::PointCloud<pcl::PointXYZRGB>);
//    *show = *result+*full_cloud;
//    viewer.showCloud(show);
//    while (!viewer.wasStopped ())
//    {
//    }








}