//cpp
#include <vector>
#include <iostream>
#include <string>
//driveworks
#include <dw/core/Context.h>
#include <dw/icp/icp.h>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
//eigen
#include <Eigen/Dense>



typedef dwLidarPointXYZI dwPoint;
typedef std::vector<dwPoint> dwPCD;
using namespace std;
using namespace Eigen;

std::vector<dwPoint> pcd2dwpc (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
    dwPCD cdwpcd;
    cdwpcd.reserve(5000);
    for (size_t i = 0 ; i<cloud->size();i++){
        dwPoint dwp = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,1};
        cdwpcd.push_back(dwp);
    }
    return cdwpcd;
}

Eigen::Matrix4f dwt2eigent(dwTransformation& dwt){
    Eigen::Matrix4f ematrix;
    for(int8_t i = 0; i < 16 ; ++i){
        ematrix(i) = dwt.array[i] ;
    }
    return ematrix;
}

dwTransformation eigent2dwt(Eigen::Matrix4f& ematrix){
    dwTransformation deM = DW_IDENTITY_TRANSFORMATION;
    for(int8_t i = 0; i < 16 ; ++i){
        deM.array[i] = ematrix(i);
    }
    return deM;
}

int main(int argc, char **argv)
{

    dwContextHandle_t context   = DW_NULL_HANDLE;
    dwICPHandle_t  icpHandle    = DW_NULL_HANDLE;

    /**instantiate Driveworks SDK context**/
    dwContextParameters sdkParams = {};
    dwVersion sdkVersion;
    dwGetVersion(&sdkVersion);
    dwInitialize(&context, sdkVersion, &sdkParams);

    /** status **/
    std::cout << "Context of Driveworks SDK successfully initialized." <<std::endl;
    std::cout << "Version: " << sdkVersion.major << "." << sdkVersion.minor << "." << sdkVersion.patch << std::endl;
    int32_t gpuCount;
    dwContext_getGPUCount(&gpuCount, context);
    std::cout << "Available GPUs: " << gpuCount << std::endl;

    /**intial dwICP**/
    dwICPParams params{};
    params.maxPoints=30000;
    params.icpType=dwICPType::DW_ICP_TYPE_LIDAR_POINT_CLOUD;
    dwICP_initialize(&icpHandle, &params, context);
    dwICP_setMaxIterations(200, icpHandle);
    dwICP_setConvergenceTolerance(1e-5, 1e-5, icpHandle);

    /**load pcd**/
    pcl::PointCloud<pcl::PointXYZ>::Ptr fed_data_pc (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fed_model_pc (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr dataPcd (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/vickylzy/workspaceROS/MAP_BAG/yuyao/shunyuFac1.pcd",*dataPcd);
    pcl::PointCloud<pcl::PointXYZ>::Ptr modelPcd (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/vickylzy/workspaceROS/MAP_BAG/yuyao/shunyuFac2.pcd",*modelPcd);
    /**downsample**/
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize(0.05f,0.05f,0.01f);
    sor.setInputCloud(dataPcd);
    sor.filter(*fed_data_pc);
    sor.setInputCloud(modelPcd);
    sor.filter(*fed_model_pc);
    /** preform icp**/
    dwPCD dataCloud = pcd2dwpc(fed_data_pc);
    dwPCD ModelCloud = pcd2dwpc(fed_model_pc);
    dwICPIterationParams icpPatams{};
    icpPatams.sourcePts =  dataCloud.data();
    icpPatams.targetPts =  ModelCloud.data();
    icpPatams.nSourcePts = dataCloud.size();
    icpPatams.nTargetPts = ModelCloud.size();
    dwTransformation icpPriorPose = DW_IDENTITY_TRANSFORMATION;
    icpPatams.initialSource2Target = &icpPriorPose;
    dwTransformation resultPose;

    clock_t start = clock();
    dwICP_optimize(&resultPose, &icpPatams, icpHandle);
    float64_t icpTime = 1000*(float64_t(clock()) - start ) / CLOCKS_PER_SEC;

    /**Get some stats about the ICP perforlmance**/
    dwICPResultStats icpResultStats;
    dwICP_getLastResultStats(&icpResultStats, icpHandle);
    cout << "ICP Time: " << icpTime << "ms" << endl
         << "Number of Iterations: " << icpResultStats.actualNumIterations << endl
         << "Number of point correspondences: " << icpResultStats.numCorrespondences << endl
         << "RMS cost: " << icpResultStats.rmsCost << endl
         << "Inlier fraction: " << icpResultStats.inlierFraction << endl
         << "ICP Spin Transform: " <<endl <<dwt2eigent(resultPose)<< endl;

    /**viewer**/
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Viewer"));

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*dataPcd, *transformed_cloud, dwt2eigent(resultPose));
    viewer->addPointCloud(modelPcd,"model");
    viewer->addPointCloud(transformed_cloud,"trans_data");
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    /**release Driveworks SDK context**/
    dwICP_release(&icpHandle);
    dwRelease(&context);
    return 0;
}