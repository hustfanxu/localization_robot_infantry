
#ifndef DISCRIPTOR_HPP
#define DISCRIPTOR_HPP

#include <Eigen/Dense>

#include <mutex>
#include <memory>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>

using namespace Eigen;
using pointT=pcl::PointXYZ;

namespace prm_localization {

    class discriptor {
    //descriptors max num
    static const int MAX_DES = 9000;

    public:
        /**
         * extractEig feature from cloud with spec gridstep
         * @param src_cloud
         * @param gridstep
         */
        discriptor(const pcl::PointCloud<pointT>::Ptr src_cloud, float gridstep) : gridstep(gridstep) {

            //time test
            auto start = std::chrono::system_clock::now();
            //down sample
            boost::shared_ptr<pcl::VoxelGrid<pointT>> voxelgrid (new pcl::VoxelGrid<pointT>());
            voxelgrid->setLeafSize(gridstep,gridstep,gridstep);
            voxelgrid->setInputCloud(src_cloud);
            pcl::PointCloud<pointT>::Ptr down_src_cloud (new pcl::PointCloud<pointT>());
            voxelgrid->filter(*down_src_cloud);
            //ready grid step
            boost::shared_array< float > gridsteps (new float_t[4]);
            for (int i = 0; i < 4 ; ++i) {
                gridsteps[i] = (i+1)/2*gridstep;
            }
            //select descriptor points
            select_desp_index.resize(MAX_DES/20);
            select_desp_index.clear();
            std::vector<std::vector<int>>  srcIdx;
            srcIdx.resize(MAX_DES/20);
            srcIdx.clear();
            range_search(src_cloud,down_src_cloud,select_desp_index,srcIdx,gridsteps[0]);
            //
            auto finished = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finished-start);
            int a = 1;
//            std::cout<<

        }
        /**
         * search near points in @src_cloud with @radius for @down_cloud
         * @param src_cloud
         * @param down_cloud
         * @param srcSeedIndex
         * @param srcIdx
         * @param radius
         */
        void range_search(const pcl::PointCloud<pointT>::Ptr src_cloud,const pcl::PointCloud<pointT>::Ptr down_cloud,
                std::vector<uint32_t > &srcSeedIndex,std::vector<std::vector<int>> &srcIdx,float radius){
            pcl::KdTreeFLANN< pointT > kdTreeFlann;
            kdTreeFlann.setInputCloud(src_cloud);
            std::vector<int> srcIdxPer;
            std::vector<float> pointRadiusSquaredDistance;
            for (int i=0;i<down_cloud->size();++i){
                if(kdTreeFlann.radiusSearch(down_cloud->points[i],radius,srcIdxPer,pointRadiusSquaredDistance)>0){
                    if (srcIdxPer.size()>=10) {
                        srcSeedIndex.push_back(i);
                        srcIdx.push_back(srcIdxPer);
                    }
                }
            }
            psize = srcSeedIndex.size();
        }



        
    private:

        boost::shared_ptr<Matrix<double ,9,-1,0,9,MAX_DES>> despcrip;
        boost::shared_ptr<Matrix<double ,3,-1,0,3,MAX_DES>> position;
        boost::shared_ptr<Matrix<double ,12,-1,0,12,MAX_DES>> norm;
        std::vector< uint32_t >select_desp_index;
        float gridstep;
        double psize;


    public:
        const boost::shared_ptr<Matrix<double, 9, -1, 0, 9, MAX_DES>> &getDespcrip() const {
            return despcrip;
        }

        const boost::shared_ptr<Matrix<double, 3, -1, 0, 3, MAX_DES>> &getPosition() const {
            return position;
        }

        const boost::shared_ptr<Matrix<double, 12, -1, 0, 12, MAX_DES>> &getNorm() const {
            return norm;
        }

        const std::vector<uint32_t> &getSelectDespIndex() const {
            return select_desp_index;
        }

        float getGridstep() const {
            return gridstep;
        }

        double getPsize() const {
            return psize;
        }


    };

}

#endif //DISCRIPTOR_HPP
