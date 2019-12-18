
#ifndef TRANSFORM_UTILITY
#define TRANSFORM_UTILITY

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

/**function for conversation**/

using namespace Eigen;

Matrix3f euler2rot(const float x_pi,const float y_pi,const float z_pi){
    Matrix3f m;
    m = AngleAxisf(z_pi, Vector3f::UnitZ())
        * AngleAxisf(y_pi, Vector3f::UnitY())
        * AngleAxisf(x_pi, Vector3f::UnitX());
    return m;
}
Vector3f rot2euler(const Matrix3f& m){
    return m.eulerAngles(2,1,0);
}

Quaternionf euler2quat(const float xr_pi,const float yp_pi,const float zy_pi){
    Quaternionf q;
    q = AngleAxisf(xr_pi, Vector3f::UnitX())
        * AngleAxisf(yp_pi, Vector3f::UnitY())
        * AngleAxisf(zy_pi, Vector3f::UnitZ());
    return q;
}

Quaternionf rot2quat(const Eigen::Matrix4f& pose){
    Eigen::Quaternionf quat(pose.block<3,3>(0, 0));
    quat.normalize();
    return quat;
}

Vector3f quat2euler(const Quaternionf& quaternionf){
    return quaternionf.toRotationMatrix().eulerAngles(0,1,2);
}

Matrix3f quat2rot(const Quaternionf& quaternionf){
    return quaternionf.toRotationMatrix();
}

nav_msgs::Odometry rotm2odometry(const Eigen::Matrix4f& pose ,const ros::Time& stamp,const std::string& frame_id, const std::string& child_frame_id){
    Quaternionf q = rot2quat(pose);
    nav_msgs::Odometry odometry;
    odometry.header.frame_id=frame_id;
    odometry.child_frame_id = child_frame_id;
    odometry.header.stamp = stamp;
    odometry.pose.pose.position.x = pose(0,3);
    odometry.pose.pose.position.y = pose(1,3);
    odometry.pose.pose.position.z = pose(2,3);
    odometry.pose.pose.orientation.x=q.x();
    odometry.pose.pose.orientation.y=q.y();
    odometry.pose.pose.orientation.z=q.z();
    odometry.pose.pose.orientation.w=q.w();
    return odometry;
}
Matrix4f odom2rotm(const nav_msgs::OdometryConstPtr& odom_msg){
    Matrix4f m4f;
    m4f.setIdentity();
    Quaternionf qf;
    m4f(0,3) = odom_msg->pose.pose.position.x;
    m4f(1,3) = odom_msg->pose.pose.position.y;
    m4f(2,3) = odom_msg->pose.pose.position.z;
    qf.x() = odom_msg->pose.pose.orientation.x;
    qf.y() = odom_msg->pose.pose.orientation.y;
    qf.z() = odom_msg->pose.pose.orientation.z;
    qf.w() = odom_msg->pose.pose.orientation.w;
    m4f.block(0,0,3,3) = quat2rot(qf);
    return m4f;
}
Matrix4f odom2rotm(const nav_msgs::Odometry& odom_msg){
    Matrix4f m4f;
    m4f.setIdentity();
    Quaternionf qf;
    m4f(0,3) = odom_msg.pose.pose.position.x;
    m4f(1,3) = odom_msg.pose.pose.position.y;
    m4f(2,3) = odom_msg.pose.pose.position.z;
    qf.x() = odom_msg.pose.pose.orientation.x;
    qf.y() = odom_msg.pose.pose.orientation.y;
    qf.z() = odom_msg.pose.pose.orientation.z;
    qf.w() = odom_msg.pose.pose.orientation.w;
    m4f.block(0,0,3,3) = quat2rot(qf);
    return m4f;
}
Matrix4f tftransform2rotm(const tf::StampedTransform& stampedTransform){
    Matrix4f m4f;
    m4f.setIdentity();
    tf::Quaternion tfquat =  stampedTransform.getRotation();
    tf::Vector3 tfvec3 = stampedTransform.getOrigin();
    Eigen::Quaternionf eigenQuat(tfquat.w(),tfquat.x(),tfquat.y(),tfquat.z());
    m4f.block(0,0,3,3) = quat2rot(eigenQuat);
    m4f(0,3)= tfvec3.x();
    m4f(1,3)= tfvec3.y();
    m4f(2,3)= tfvec3.z();
    return m4f;
}


/**
 * from hdl_localization
 * @brief convert a Eigen::Matrix to TransformedStamped
 * @param stamp           timestamp
 * @param pose            pose matrix
 * @param frame_id        frame_id
 * @param child_frame_id  child_frame_id
 * @return transform
 */
geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3,3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
}

#endif //TRANSFORM_UTILITY
