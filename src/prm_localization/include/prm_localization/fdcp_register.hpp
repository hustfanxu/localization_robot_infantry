#ifndef FDCP_REGISTER_HPP
#define FDCP_REGISTER_HPP

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <prm_localization/discriptor.hpp>
namespace prm_localization {
/**
 * @brief do prm_pointCloud_registration
 */
    class Fdcp_register{
    public:
        using PointT = pcl::PointXYZ;

        Fdcp_register(const ros::Time &stamp ,pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, double >::Ptr &registrion) :
            registrion(registrion),
            curr_stamp(stamp)

        {

        }

    private:
        ros::Time curr_stamp;
        pcl::Registration<PointT,PointT,double>::Ptr registrion;
    };

}
#endif //FDCP_REGISTER_HPP