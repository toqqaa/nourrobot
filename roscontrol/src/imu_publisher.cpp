#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nour_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

// TF broadcaster (optional - only if you need to publish the transform)
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher imu_pub;
nour_msgs::Imu imu_data;

// Frame ID configuration
std::string imu_frame_id = "imu_link";  // Make this configurable

void IMUCallback(const std_msgs::Float32MultiArray::ConstPtr &raw_imu_msg)
{
    // Process IMU data
    double yaw = raw_imu_msg->data[0];
    double pitch = raw_imu_msg->data[1];
    double roll = raw_imu_msg->data[2];

    // Create message
    nour_msgs::Imu imu_msg;
    
    // Set header
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = imu_frame_id;

    // Set linear acceleration (example values)
    geometry_msgs::Vector3 linear_acc;
    linear_acc.x = 0;  // Replace with actual values
    linear_acc.y = 0;
    linear_acc.z = 1;
    imu_msg.linear_acceleration = linear_acc;

    // Set angular velocity (example values)
    geometry_msgs::Vector3 angular_vel;
    angular_vel.x = 0;  // Replace with actual values
    angular_vel.y = 0;
    angular_vel.z = 0;
    imu_msg.angular_velocity = angular_vel;

    // Set magnetic field (example values)
    geometry_msgs::Vector3 mag_field;
    mag_field.x = 0;  // Replace with actual values
    mag_field.y = 0;
    mag_field.z = 0;
    imu_msg.magnetic_field = mag_field;

    // Publish the message
    imu_pub.publish(imu_msg);

    // Optional: Publish TF transform
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "base_link";  // Parent frame
    transform.child_frame_id = imu_frame_id;
    
    // Set transform (identity by default)
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    br.sendTransform(transform);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;
    
    // Get frame_id from parameter server if available
    nh.param<std::string>("imu_frame_id", imu_frame_id, "imu_link");

    imu_pub = nh.advertise<nour_msgs::Imu>("imu_data", 10);
    ros::Subscriber imu_sub = nh.subscribe("raw_imu", 10, IMUCallback);

    ros::spin();
    return 0;
}
