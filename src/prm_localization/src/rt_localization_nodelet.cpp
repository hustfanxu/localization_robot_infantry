#include "ros/ros.h"
// #include "time.h"
#include "std_msgs/String.h"
// ros_time
#include <ros/timer.h>
// ros_msg
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
//eigen
#include <Eigen/Dense>
//tf
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//nodelet
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
//driveworks
//#include <dw/core/Context.h>
//#include <dw/icp/icp.h>
//cpp
#include <ctime>
#include <limits.h>
#include <math.h>
#include <mutex>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <boost/filesystem.hpp>
//untility
#include <prm_localization/transform_utility.hpp>
#include <prm_localization/save_data.h>



namespace rt_localization_ns{
    using namespace std;
//    typedef dwLidarPointXYZI dwPoint;
//    typedef std::vector<dwPoint> dwPCD;

    class RealTime_Localization: public nodelet::Nodelet {
    public:
        RealTime_Localization(){
        }
        virtual  ~RealTime_Localization(){
        }

        void onInit() override {


            /**init**/
            nh = getNodeHandle();
            mt_nh = getMTNodeHandle();
            private_nh = getPrivateNodeHandle();
            /**parameter**/
            curr_fitness=0;
            regis_threshlod=1;
            trim_low = private_nh.param<float>("trim_low", 0.0f);
            lidar_height= private_nh.param<float>("lidar_height", 1.85f);
            trim_high = private_nh.param<float>("trim_high", 4.0f);
            farPointThreshold = private_nh.param<float>("farPointThreshold", 30.0f);
            nearPointThreshold = private_nh.param<float>("nearPointThreshold", 1.4f); //pioneer carself
            lp_odom_rate = private_nh.param<float>("lp_odom_rate", 0.0f);
            regis_hard_threshold = private_nh.param<float>("regis_hard_threshold", 10.0f);
            auto init_x = private_nh.param<float>("init_x", 0.0f);
            auto init_y = private_nh.param<float>("init_y", 0.0f);
            auto init_yaw = private_nh.param<double>("init_yaw", 0.0);
            auto icpMaximumIterations = private_nh.param<int>("icpMaximumIterations", 50);
            auto icpRANSACOutlierRejectionThreshold = private_nh.param<float>("icpRANSACOutlierRejectionThreshold", 0.04f);
            auto icpTransformationEpsilon = private_nh.param<float>("icpTransformationEpsilon", 1e-5);
            auto downSampleSize = private_nh.param<float>("downSampleSize", 0.05f);
            auto TransformationEpsilon = private_nh.param<float>("TransformationEpsilon", 0.01f);
            auto ndt_resolution = private_nh.param<float>("ndt_resolution", 1.0f);
            use_GPU_ICP = private_nh.param<bool>("use_GPU_ICP", false);
            flag_slam_insout = private_nh.param<bool>("flag_slam_insout", false);
            map_tf = private_nh.param<std::string>("map_tf", "map");
            base_lidar_tf = private_nh.param<std::string>("base_lidar_tf", "velodyne");
            base_foot_tf = private_nh.param<std::string>("base_foot_tf", "ins_center_frame");

            thresholdTimes = 100;
            fitness_buffer.set_capacity(10);
            cout<<"fitness buffer success"<<endl;
            velosity_x=velosity_y=velosity_yaw=0;
            localmap_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

            //init_pose
            curr_pose.setIdentity();
            curr_pose.block(0,0,3,3) = euler2rot(0,0,init_yaw);
            curr_pose(0,3) =init_x;
            curr_pose(1,3) =init_y;
            curr_pose(2,3) =lidar_height;
            curr_pose_stamp =ros::Time(0.001);
            ins2VeloMli.setIdentity();
//            ins2VeloMli << 0.999501,0.0304109,0.00848289,-2.08752,
//                    -0.0303736,    0.999529, -0.00448464,   0.0823302,
//                    -0.00861528,  0.00422475,    0.999954,    -1.45733,
//                    0,           0,           0,           1;
//            cout<<"ins_center_2_velo_middle :"<<endl<<ins2VeloMli<<endl;

            if (use_GPU_ICP){
//                context     = DW_NULL_HANDLE;
//                icpHandle   = DW_NULL_HANDLE;
//                dwContextParameters sdkParams = {};
//                dwVersion sdkVersion;
//                dwGetVersion(&sdkVersion);
//                dwInitialize(&context, sdkVersion, &sdkParams);
//                /** status **/
//                std::cout << "Context of Driveworks SDK successfully initialized." <<std::endl;
//                std::cout << "Version: " << sdkVersion.major << "." << sdkVersion.minor << "." << sdkVersion.patch << std::endl;
//                int32_t gpuCount;
//                dwContext_getGPUCount(&gpuCount, context);
//                std::cout << "Available GPUs: " << gpuCount << std::endl;
//                /**intial dwICP**/
//                dwICPParams params{};
//                params.maxPoints=32767;
//                params.icpType=dwICPType::DW_ICP_TYPE_LIDAR_POINT_CLOUD;
//                dwICP_initialize(&icpHandle, &params, context);
//                dwICP_setMaxIterations(icpMaximumIterations, icpHandle);
//                dwICP_setConvergenceTolerance(icpTransformationEpsilon, icpTransformationEpsilon, icpHandle);
            }
            else {
                //ndt_omp
               pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt(
                       new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
               ndt->setTransformationEpsilon(TransformationEpsilon);
               ndt->setResolution(ndt_resolution);
               ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
               registration = ndt;
                //registration-icp
            // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr regis (new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
            // registration = regis;
            // registration->setMaximumIterations(icpMaximumIterations);
            // registration->setRANSACOutlierRejectionThreshold(icpRANSACOutlierRejectionThreshold);
            // registration->setMaxCorrespondenceDistance(icpRANSACOutlierRejectionThreshold*100);
            // registration->setTransformationEpsilon(icpTransformationEpsilon);
//        registration.set
            }
            /**sub and pub**/
//            odom_pub = nh.advertise<nav_msgs::Odometry>(map_tf,50);
            odom_pub = nh.advertise<nav_msgs::Odometry>("/localization/odom",50);
            points_suber = mt_nh.subscribe("/velodyne_points",1,&RealTime_Localization::points_callback,this);
            localmap_suber =  nh.subscribe("/localization/localmap",1,&RealTime_Localization::localmap_callback,this);
            curr_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/localization/registered_pointCloud",5);
            // pub prefict transform (optinal)
            lp_odom_pub = nh.advertise<nav_msgs::Odometry>("/localization/lp_odom", 10);
            if(lp_odom_rate != 0) {
                lp_timer = nh.createTimer(ros::Duration(1.0f / lp_odom_rate), &RealTime_Localization::lp_odom_callback,
                                          this);
            }
            service = nh.advertiseService("save_curr",&RealTime_Localization::save_curr_func,this);
            //add for debug regis_input
            predict_now_odom = nh.advertise<nav_msgs::Odometry>("/localization/regis_in_odom",50);
            /**utility param**/
            downSampler.setLeafSize(downSampleSize,downSampleSize,downSampleSize);

            NODELET_INFO("realTime_localization_nodelet initial completed");


        }

    private:

        bool save_curr_func(prm_localization::save_data::Request &req,prm_localization::save_data::Response &res){
            boost::filesystem::path dir( req.sace_path_dir);
            if(boost::filesystem::create_directory(dir)) {
                std::cout << "make a dir on "<< req.sace_path_dir << endl;
            }
            pcl::io::savePCDFileASCII(req.sace_path_dir+"/data_cloud.pcd",*registered_cloud);
            pcl::io::savePCDFileASCII(req.sace_path_dir+"/localmap_cloud.pcd",*localmap_cloud);
            return true;
        }


        /**lp_odom**/
        void lp_odom_callback(const ros::TimerEvent& event){
            if (curr_fitness > thresholdTimes*regis_threshlod || curr_fitness > regis_hard_threshold ){
                return;
            }
            Matrix4f predict_pose = predict(curr_pose,curr_pose_stamp,event.current_real);
            Matrix4f predict_ins_pose = predict_pose * ins2VeloMli;
            nav_msgs::Odometry odometry  = rotm2odometry(predict_ins_pose,event.current_real,map_tf,base_foot_tf);
            odometry.pose.covariance[0]=curr_fitness;
            odometry.pose.covariance[1]=regis_threshlod;
            transformBroadcaster.sendTransform(matrix2transform(event.current_real,predict_ins_pose,map_tf,base_foot_tf));
            lp_odom_pub.publish(odometry);

        }

        void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg){

            // double begin = ros::Time::now().toSec();
            if(localmap_cloud->empty()){
                NODELET_INFO("waiting for map!");
                return;
            }
            // convert ros msg
            pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*points_msg, *curr_cloud);
            if(curr_cloud->empty()){
                NODELET_INFO("lidar provide emptry cloud!");
                return;
            }
            //downsample
            downSampler.setInputCloud(curr_cloud);
            downSampler.filter(*filtered_cloud);
            //trim and 2d
            auto flat_cloud  = trimInputCloud(filtered_cloud,nearPointThreshold,farPointThreshold, trim_low, trim_high);
            //check lidar sight
            if(flat_cloud->size()<300){
                NODELET_INFO("lidar lose sight!");
                //you should set a FLOAT_MAX instead of 5000 but not just that
                curr_fitness=5000;
                return;
            }
            //pc register  (with predict pose by former movement)
            Matrix4f predict_pose = predict(curr_pose,curr_pose_stamp,points_msg->header.stamp);
            if (curr_fitness <= thresholdTimes*regis_threshlod && curr_fitness<regis_hard_threshold ) {
                nav_msgs::Odometry odometry;
                if(flag_slam_insout)
                {
                    odometry = rotm2odometry(predict_pose * ins2VeloMli ,points_msg->header.stamp,map_tf,base_foot_tf);
                }
                else
                {
                    odometry  = rotm2odometry(predict_pose,points_msg->header.stamp,map_tf,base_lidar_tf);
                }
                odometry.pose.covariance[0]=curr_fitness;
                odometry.pose.covariance[1]=regis_threshlod;
//                odometry.twist.twist.linear.x=velosity_x;
//                odometry.twist.twist.linear.y=velosity_y;
//                odometry.twist.twist.angular.z = velosity_yaw;
                predict_now_odom.publish(odometry);
            }
                /*  "predict_pose" for lidar predict
                 *  "odom_pose" for imu odometry or karmanfilter
                 *  "curr_pose" for none
                 */
            Matrix4f transform;
            if (curr_fitness > thresholdTimes*regis_threshlod  || curr_fitness > regis_hard_threshold){
                transform = pc_register(flat_cloud, localmap_cloud, predict_pose);
            } else
                transform = pc_register(flat_cloud, localmap_cloud, predict_pose);
            //abandon old regis
            {
            lock_guard<mutex> lockGuard(curr_pose_mutex);
            if(points_msg->header.stamp<curr_pose_stamp){
                NODELET_INFO("abandon former regis result");
                return;
            }
            }
            //judge bad regis
            if (curr_fitness > thresholdTimes*regis_threshlod || curr_fitness > regis_hard_threshold){
                NODELET_INFO("cannot match currcloud with lidar %d points, fitness:\t%f",(int)flat_cloud->size(),curr_fitness);
                return;
            }
            fitness_buffer.push_back(curr_fitness);
            regis_threshlod =  std::accumulate(fitness_buffer.begin(),fitness_buffer.end(),0.0f)/fitness_buffer.size();
            NODELET_INFO("regis_threshlod %d:\t%f",thresholdTimes,thresholdTimes*regis_threshlod);
//            cout<<"regis_threshlod*"<<thresholdTimes<<":\t"<<thresholdTimes*regis_threshlod<<endl;
            //change status
            update_velocity(curr_pose,transform,curr_pose_stamp,points_msg->header.stamp);
            curr_pose=transform;
//            NODELET_INFO("time to former stamp = %f seconds",points_msg->header.stamp.toSec()-curr_pose_stamp.toSec());
            curr_pose_stamp = points_msg->header.stamp;


            //publish tf
            transformBroadcaster.sendTransform(matrix2transform(points_msg->header.stamp,curr_pose,map_tf,base_lidar_tf));
            transformBroadcaster.sendTransform(matrix2transform(points_msg->header.stamp,transform *ins2VeloMli,map_tf,base_foot_tf));
            //publish (transformed_ins_odom)odom
            nav_msgs::Odometry odom;
            if(flag_slam_insout){
                odom = rotm2odometry( transform *ins2VeloMli,points_msg->header.stamp,map_tf,base_foot_tf)  ;
                lp_odom_pub.publish(rotm2odometry(transform,points_msg->header.stamp,map_tf,base_lidar_tf));
            }else{
                odom = rotm2odometry(transform,points_msg->header.stamp,map_tf,base_lidar_tf);
                lp_odom_pub.publish(rotm2odometry( transform *ins2VeloMli,points_msg->header.stamp,map_tf,base_foot_tf) );
            }
            odom.pose.covariance[0]=curr_fitness;
            odom.pose.covariance[1]=regis_threshlod;
//            odom.twist.twist.linear.x=velosity_x;
//            odom.twist.twist.linear.y=velosity_y;
//            odom.twist.twist.angular.z = velosity_yaw;
            odom_pub.publish(odom);
            // double end = ros::Time::now().toSec();
            // cout << "process time" << end-begin << endl;


            //publish cloud  (optional)
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZ>());
            //raw cloud
//            pcl::transformPointCloud(*curr_cloud,*transformedCloud,transform);
            //process cloud
            pcl::transformPointCloud(*flat_cloud,*transformedCloud,transform);
            pcl_conversions::toPCL(points_msg->header,transformedCloud->header);
            transformedCloud->header.frame_id = map_tf;
            curr_pointcloud_pub.publish(transformedCloud);
            registered_cloud = transformedCloud;

        }


        void localmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg){
            localmap_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*points_msg, *localmap_cloud);
            kdtree.setInputCloud(localmap_cloud);
        }

        pcl::PointCloud<pcl::PointXYZ>::ConstPtr trimInputCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                                                                const float disNear, const float disFar, const float low, const float high)  {

//            clock_t start = clock();

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTrimmed (new pcl::PointCloud<pcl::PointXYZ>());
            cloudTrimmed->reserve(cloud->size()/2);
            std::copy_if(cloud->begin(),cloud->end(),std::back_inserter(cloudTrimmed->points),
                         [&](const pcl::PointXYZ& p){
                             double d = p.getVector3fMap().norm();
                             return d > disNear && d < disFar && p.z > low && p.z <high;
                         }
            );
            cloudTrimmed->width = cloudTrimmed->points.size();
            cloudTrimmed->height = 1;
            cloudTrimmed->header = cloud->header;
//            clock_t end = clock();
//            time_sum+=(double)(end  - start) / CLOCKS_PER_SEC;
//            NODELET_INFO("curr avg trim time = %f seconds",time_sum/(++trim_time));
            return cloudTrimmed;
        }



        void update_velocity(const Eigen::Matrix4f& original_pose , const Eigen::Matrix4f& registered_pose,const ros::Time& oringal_time, const ros::Time& curr_time){
            //not compute speed when recovery
            if (curr_time == oringal_time)
                return;
            Eigen::Vector3f original_oritetion = rot2euler(original_pose.block(0,0,3,3));
            Eigen::Vector3f registered_oritetion = rot2euler(registered_pose.block(0,0,3,3));
            double time_colleasep = (curr_time.toSec()-oringal_time.toSec());
            velosity_yaw = calculate_dradius(original_oritetion,registered_oritetion) /time_colleasep;
            velosity_x = (registered_pose(0,3)-original_pose(0,3))/time_colleasep;
            velosity_y = (registered_pose(1,3)-original_pose(1,3))/time_colleasep;
        }

        float exart_yaw(const Eigen::Vector3f& vec) const {
            float theta;
            if(fabs(vec(1))>M_PI/2)  theta = -(M_PI - vec(0));
            else theta = vec(0);
            return theta;
        }

        float calculate_dradius(const  Eigen::Vector3f& start_vec , const  Eigen::Vector3f& end_vec) const{
            float start = exart_yaw(start_vec);
            float end = exart_yaw(end_vec);
            float degree_raw = end-start;
            float degree_inv = 2*M_PI-fabs(degree_raw);
            float sig = end > start ? -1 : 1;
            return fabs(degree_raw)<fabs(degree_inv) ? degree_raw : sig *degree_inv;
        }

        Eigen::Matrix4f predict(const Eigen::Matrix4f& original_pose, const ros::Time& oringal_time, const ros::Time& curr_time) const {
            double time_collapse = (curr_time.toSec()-oringal_time.toSec());
            Eigen::Matrix4f future_pose;
            future_pose.setIdentity();
            Eigen::Vector3f original_oritetion = rot2euler(original_pose.block(0,0,3,3));
            float tar_orit =fmod (exart_yaw(original_oritetion)+velosity_yaw*time_collapse+M_PI , 2* M_PI) -M_PI;
            future_pose.block(0,0,3,3) = euler2rot(0,0,tar_orit);
            future_pose(0,3) = original_pose(0,3)+velosity_x*time_collapse;
            future_pose(1,3) = original_pose(1,3)+velosity_y*time_collapse;
            future_pose(2,3) = original_pose(2,3);
            return future_pose;
        }

        Eigen::Matrix4f pc_register (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& curr_cloud,const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& local_map,const Eigen::Matrix4f& initial_matrix){
            if (curr_cloud->empty()){
                NODELET_WARN(" trimed a empty cloud !");
                return initial_matrix;
            }else if(local_map->empty()){
                NODELET_WARN(" localmap updating !");
                return initial_matrix;
            }
            if (use_GPU_ICP){
//
//                dwPCD dataCloud = pcd2dwpc(curr_cloud);
//                dwPCD ModelCloud = pcd2dwpc(local_map);
//                dwICPIterationParams icpPatams{};
//                icpPatams.sourcePts =  dataCloud.data();
//                icpPatams.targetPts =  ModelCloud.data();
//                icpPatams.nSourcePts = dataCloud.size();
//                icpPatams.nTargetPts = ModelCloud.size();
//                cout<<"curr points:\t"<<icpPatams.nSourcePts<<endl;
//                cout<<"map points:\t"<<icpPatams.nTargetPts<<endl;
//                if(icpPatams.nTargetPts>32767 ||icpPatams.nSourcePts>32767) {
//                    throw std::runtime_error("ERROR : points num cannot be large that 32767");
//                }
//                cout<<"icpPatams.nTargetPts\t"<<icpPatams.nTargetPts<<endl;
//                cout<<"icpPatams.nSourcePts\t"<<icpPatams.nSourcePts<<endl;
//                dwTransformation icpPriorPose = eigent2dwt(initial_matrix);
//                icpPatams.initialSource2Target = &icpPriorPose;
//                dwTransformation resultPose;
//                clock_t start = clock();
//                dwICP_optimize(&resultPose, &icpPatams, icpHandle);
//                clock_t end = clock();
//                NODELET_INFO("registration regis time = %f seconds",(double)(end  - start) / CLOCKS_PER_SEC);
//
//                /**Get some stats about the ICP perforlmance**/
////                dwICPResultStats icpResultStats;
////                dwICP_getLastResultStats(&icpResultStats, icpHandle);
////                cout << "Number of Iterations: " << icpResultStats.actualNumIterations << endl
////                     << "Number of point correspondences: " << icpResultStats.numCorrespondences << endl
////                     << "RMS cost: " << icpResultStats.rmsCost << endl
////                     << "Inlier fraction: " << icpResultStats.inlierFraction << endl
////                     << "ICP Spin Transform: " <<endl <<dwt2eigent(resultPose)<< endl;
//
//                return dwt2eigent(resultPose);
            } else{
                //registration
                registration->setInputTarget(local_map);
                registration->setInputSource(curr_cloud);


                pcl::PointCloud<pcl::PointXYZ> result_cloud ;
                //registration start
                clock_t start = clock();
                registration->align(result_cloud,initial_matrix);//,curr_pose
                clock_t end = clock();
                NODELET_INFO("registration regis time = %f seconds",(double)(end  - start) / CLOCKS_PER_SEC);
                curr_fitness = registration->getFitnessScore();
                NODELET_INFO("fitness score = %f ",registration->getFitnessScore());
                // test if regis wrong
//                if (curr_cloud->size()==5761)curr_fitness=5000;
                //
                return registration->getFinalTransformation();
//        Vector3f euler = rot2euler(transform.block(0,0,3,3));
//        NODELET_INFO("x = %f, y = %f, theta = %f ",transform(0,3),transform(1,3),euler(2));
            }
        }

//        std::vector<dwPoint> pcd2dwpc (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
//            dwPCD cdwpcd;
//            cdwpcd.reserve(5000);
//            for (size_t i = 0 ; i<cloud->size();i++){
//                dwPoint dwp = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,1};
//                cdwpcd.push_back(dwp);
//            }
//            return cdwpcd;
//        }
//
//        Eigen::Matrix4f dwt2eigent(const dwTransformation& dwt){
//            Eigen::Matrix4f ematrix;
//            for(int8_t i = 0; i < 16 ; ++i){
//                ematrix(i) = dwt.array[i] ;
//            }
//            return ematrix;
//        }
//
//        dwTransformation eigent2dwt(const Eigen::Matrix4f& ematrix){
//            dwTransformation deM = DW_IDENTITY_TRANSFORMATION;
//            for(int8_t i = 0; i < 16 ; ++i){
//                deM.array[i] = ematrix(i);
//            }
//            return deM;
//        }

    private:

        //ros node handle
        ros::NodeHandle nh;
        ros::NodeHandle mt_nh;
        ros::NodeHandle private_nh;
        // drivework handle
//        dwContextHandle_t context;
//        dwICPHandle_t  icpHandle;
        // para
        float trim_low;
        float lidar_height;
        float trim_high;
        float farPointThreshold;
        float nearPointThreshold;
        float regis_hard_threshold;
        Eigen::Matrix4f curr_pose;
        ros::Time curr_pose_stamp;
        double velosity_x;
        double velosity_y;
        double velosity_yaw;
        bool use_GPU_ICP;
        double curr_fitness;
        float regis_threshlod;
        int thresholdTimes;
        Eigen::Matrix4f ins2VeloMli;
        bool flag_slam_insout;
        float lp_odom_rate;

        // suber and puber
        ros::Publisher curr_pointcloud_pub;
//        ros::Subscriber odom_suber;
        ros::Subscriber points_suber;
        ros::Subscriber localmap_suber;
        ros::Publisher odom_pub;
        ros::Publisher predict_now_odom;
        ros::Publisher lp_odom_pub;
        //tf
        ros::Timer lp_timer;
        tf::TransformBroadcaster transformBroadcaster;
        tf::TransformListener transformListener;
        string map_tf;
        string base_lidar_tf;
        string base_foot_tf;

        // clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr localmap_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr registered_cloud;
        // utility
        pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;
        pcl::VoxelGrid<pcl::PointXYZ> downSampler;
        pcl::KdTreeFLANN< pcl::PointXYZ > kdtree;
        std::mutex curr_pose_mutex;
//        std::mutex imu_odom_data_mutex;
        // buffer
        boost::circular_buffer<float> fitness_buffer;
        // service
        ros::ServiceServer service ;






    };


}
PLUGINLIB_EXPORT_CLASS(rt_localization_ns::RealTime_Localization, nodelet::Nodelet)

//                NODELET_INFO("sec:%i,nsec:%i",points_msg->header.stamp.sec,points_msg->header.stamp.nsec);
//                NODELET_INFO("sec:%i,nsec:%i",curr_pose_stamp.sec,curr_pose_stamp.nsec);

//        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration;

//            cout<<"odom_pose: "<<endl<<odom_pose<<endl;
//            cout<<"calculate_pose: "<<endl<<transform<<endl;


/**select closest imu odom pose (bad performance)**/
//            Matrix4f odom_pose;
//            if (!imu_odom_data.empty())
//            {
//                for (boost::circular_buffer<nav_msgs::OdometryConstPtr>::const_iterator i = imu_odom_data.end() - 1;
//                     i != imu_odom_data.begin(); i--) {
//                    nav_msgs::Odometry odometry = **i;
//                    NODELET_INFO("pass a odom");
//                    if (odometry.header.stamp < points_msg->header.stamp) {
//                        Quaternionf q(odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
//                                      odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z);
//                        odom_pose.block(0, 0, 3, 3) = quat2rot(q);
//                        odom_pose(0, 3) = odometry.pose.pose.position.x;
//                        odom_pose(1, 3) = odometry.pose.pose.position.y;
//                        odom_pose(2, 3) = curr_pose(2,3);
//                        odometry.pose.pose.position.z=curr_pose(2,3);
//                        predict_now_odom.publish(odometry);
//                       odom_pose;
//                        break;
//                    }
//                }
//
//            }

/** multithread on point_callback**/
//{
//lock_guard<mutex> lockGuard(curr_pose_mutex);
//if(points_msg->header.stamp>curr_pose_stamp){
//
//
//} else {
//NODELET_INFO("abandon former regis result");
//return;
//}
//}

/** lookuptransform **/
//            tf::StampedTransform ins2Velo;
//            try {
//
//                transformListener.lookupTransform("velo_middle","ins_center",ros::Time(0),ins2Velo);
//            }
//            catch (tf::TransformException ex){
//                NODELET_WARN("%s",ex.what());
//            }

/**color cloud**/
//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coloredCloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
//pcl::PointCloud<pcl::RGB>::Ptr color (new pcl::PointCloud<pcl::RGB>());
/////transcloud
//color->width = transformedCloud->width;
//color->height = 1;//result_cloud.height
//color->points.resize(color->width*color->height);
//for (size_t i=0;i<transformedCloud->width;i++){
//color->points[i].r=255;
//color->points[i].g=255;
//color->points[i].b=0;
//color->points[i].a=100;
//}
//pcl::concatenateFields(*transformedCloud,*color,*coloredCloud);
//
//
//pcl_conversions::toPCL(points_msg->header,coloredCloud->header);
//coloredCloud->header.frame_id = map_tf;
//curr_pointcloud_pub.publish(coloredCloud);