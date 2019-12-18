#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
//tf
#include <tf/transform_listener.h>
// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose2D.h>
#include <prm_localization/transform_utility.hpp>

namespace globalmap_ns {
    using namespace std;

    class GlobalmapProviderNodelet : public nodelet::Nodelet {
    public:
        GlobalmapProviderNodelet() {
        }

        virtual  ~GlobalmapProviderNodelet() {
        }

        void onInit() override {
            /**init**/
            nh = getNodeHandle();
            mt_nh = getMTNodeHandle();
            private_nh = getPrivateNodeHandle();
            /**parameter**/
            radius = private_nh.param<float>("radius", 40.0f);
            trim_low = private_nh.param<float>("trim_low", 0.0f);
            lidar_height= private_nh.param<float>("lidar_height", 1.85f);
            trim_high = private_nh.param<float>("trim_high", 4.0f);
            int mapUpdateTime =private_nh.param<int>("mapUpdateTime", 8);
            auto downsample_resolution = private_nh.param<float>("downsample_resolution", 0.05f);
            use_GPU_ICP = private_nh.param<bool>("use_GPU_ICP", false);
            std::string globalmap_pcd = private_nh.param<std::string>("global_map_pcd_path", "/home/vickylzy/WorkSPacesROS/catkin_ws/src/prm_localization/data/shunYuFactory.pcd");
            map_tf = private_nh.param<std::string>("map_tf", "map");
            base_lidar_tf = private_nh.param<std::string>("base_lidar_tf", "velodyne");
            NODELET_INFO("radius: %f",radius);
            NODELET_INFO("%s",globalmap_pcd.c_str());
            /**initial pose**/
            curr_pose.reset(new geometry_msgs::PoseStamped());
            curr_pose->pose.position.x= private_nh.param<float>("init_x", 0.0f);
            curr_pose->pose.position.y= private_nh.param<float>("init_y", 0.0f);
            curr_pose->pose.position.z= 0.0f;
            Eigen::Quaternionf quaternionf=euler2quat(0,0,private_nh.param<double>("init_yaw", 0));
            curr_pose->pose.orientation.x=quaternionf.x();
            curr_pose->pose.orientation.y=quaternionf.y();
            curr_pose->pose.orientation.z=quaternionf.z();
            curr_pose->pose.orientation.w=quaternionf.w();
            /**load map and pub once**/ //maybe add voxelgrid down sample
            full_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::io::loadPCDFile(globalmap_pcd, *full_map);
            /**pcdownsample**/
            boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> voxelgrid(new pcl::VoxelGrid<pcl::PointXYZ>());
            if (use_GPU_ICP)
                voxelgrid->setLeafSize(downsample_resolution*5, downsample_resolution*5, downsample_resolution*2);
            else
                voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
            voxelgrid->setInputCloud(full_map);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
            voxelgrid->filter(*filtered);
            full_map = filtered;
//          publish globalmap once
//            full_map->header.frame_id = "map";
//            globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap",5);
//            globalmap_pub.publish(full_map);
            /**kdtree for trimmer**/
            kdtree.setInputCloud(full_map);
            /**sub and pub**/
//            pose_suber = mt_nh.subscribe("/TOPIC_OF_ODOM",1,&GlobalmapProviderNodelet::pose_callback,this);
            localmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/localization/localmap",1, true);
            timer = nh.createTimer(ros::Duration(mapUpdateTime),&GlobalmapProviderNodelet::localmap_callback,this);
            lidar_pose_sub = nh.subscribe("/localization/odom",1,&GlobalmapProviderNodelet::pose_callback,this);
            /**publish a localmap at once**/
            ros::TimerEvent init_event;
            localmap_callback(init_event);
            NODELET_INFO("globalmap_provider_nodelet initial completed");
        }

    private:
        /**
         * update newest pose
         * @param pose_msg
         */
        void pose_callback(const nav_msgs::OdometryConstPtr& odom_msg) {
            curr_pose->pose.position.x=odom_msg->pose.pose.position.x;
            curr_pose->pose.position.y=odom_msg->pose.pose.position.y;
            curr_pose->pose.position.z=odom_msg->pose.pose.position.z;

        }
        /**
         * trim local_map with latest pose
         * @param event
         * !note:publish pointcloud<pcl::PointXYZI>
         *
         */
        void localmap_callback(const ros::TimerEvent& event) {
//            clock_t start = clock();


            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            pcl::PointXYZ searchPoint;
            searchPoint.x = curr_pose->pose.position.x;
            searchPoint.y = curr_pose->pose.position.y;
            searchPoint.z = curr_pose->pose.position.z;
            
            // NODELET_INFO("x:%f\t,y:%f\t,z:%f\t,kdtreeP:%d",searchPoint.x,searchPoint.y,searchPoint.z,(int)kdtree.getInputCloud()->size());
            pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            float z_min_threshold = lidar_height+trim_low;
            float z_max_threshold = lidar_height+trim_high;
            if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {
//                NODELET_INFO("trimmed_cloud init points_num:%ld",trimmed_cloud->width);
//                NODELET_INFO("search points_num:%ld",pointIdxRadiusSearch.size());
                trimmed_cloud->points.reserve(60000);
                for (int i : pointIdxRadiusSearch)
                {
                    if (full_map->points[i].z > z_min_threshold && full_map->points[i].z < z_max_threshold){
                    pcl::PointXYZ cpt(full_map->points[i].x,full_map->points[i].y,full_map->points[i].z);
                    trimmed_cloud->points.push_back(cpt);
                    }
                }
                trimmed_cloud->width=trimmed_cloud->points.size();
                trimmed_cloud->height = 1;

                cout<<"full_map.size()\t:"<<full_map->size()<<endl<<"trimmed_cloud->width:\t"<<trimmed_cloud->width<<endl;
            }
            trimmed_cloud->header.frame_id=map_tf;
            pcl_conversions::toPCL(curr_pose->header.stamp, trimmed_cloud->header.stamp);
            localmap_pub.publish(trimmed_cloud);
            NODELET_INFO(" local map updated on x:%f, y:%f, z:%f", searchPoint.x,searchPoint.y,searchPoint.z);
//            clock_t end = clock();
//            NODELET_INFO("trim time = %f seconds",(double)(end  - start) / CLOCKS_PER_SEC);
        }



    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr full_map;
    private:
        //ros node handle
        ros::NodeHandle nh;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;
        // suber and puber
        ros::Subscriber lidar_pose_sub;
        ros::Publisher localmap_pub;
        //tf
        string map_tf;
        string base_lidar_tf;
        // parameter

        geometry_msgs::PoseStampedPtr curr_pose;
        pcl::KdTreeFLANN< pcl::PointXYZ > kdtree;
        bool use_GPU_ICP;
        float radius;
        float lidar_height;
        float trim_low;
        float trim_high;
        // ros timer
        ros::Timer timer ;



    };

}
PLUGINLIB_EXPORT_CLASS(globalmap_ns::GlobalmapProviderNodelet, nodelet::Nodelet)

//        /**viewer**/
//    pcl::visualization::CloudViewer viewer("window");
//    viewer.showCloud(realtime_localization->full_map);
//    while (!viewer.wasStopped())
//    {
//    }