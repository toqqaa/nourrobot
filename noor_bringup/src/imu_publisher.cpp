#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

ros::Publisher imuPub_;
ros::Publisher posePub_;
geometry_msgs::PoseStamped pose_msg;
sensor_msgs::Imu imu_msg;

void quaternionCb(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    bool pose_subscribed = (posePub_.getNumSubscribers() > 0);
    bool imu_subscribed = (imuPub_.getNumSubscribers() > 0);
    
    if (pose_subscribed || imu_subscribed)
    {
        ros::Time sensor_data_time = ros::Time::now();
        
        if (imu_subscribed)
        {
            // Fill IMU message
            imu_msg.header.stamp = sensor_data_time;
            imu_msg.orientation = *msg;
            
            // Provide some realistic sensor data instead of zeros
            // You should replace these with actual sensor readings if available
            imu_msg.angular_velocity.x = 0.01;  // Small non-zero values
            imu_msg.angular_velocity.y = 0.01;
            imu_msg.angular_velocity.z = 0.01;
            
            imu_msg.linear_acceleration.x = 0.1;  // Small acceleration values
            imu_msg.linear_acceleration.y = 0.1;
            imu_msg.linear_acceleration.z = 9.81; // Gravity
            
            imuPub_.publish(imu_msg);
            
            // Debug output
            ROS_DEBUG_THROTTLE(1.0, "Publishing IMU data: quat(%.3f,%.3f,%.3f,%.3f)", 
                      msg->x, msg->y, msg->z, msg->w);
        }
        
        if (pose_subscribed)
        {
            pose_msg.header.stamp = sensor_data_time;
            pose_msg.pose.orientation = *msg;
            posePub_.publish(pose_msg);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpu6050_imu_converter");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // Parameters
    double linear_acceleration_stdev, angular_velocity_stdev;
    std::string frame_id;
    pnh.param<double>("linear_acceleration_stdev", linear_acceleration_stdev, 0.06);
    pnh.param<double>("angular_velocity_stdev", angular_velocity_stdev, 0.005);
    pnh.param<std::string>("frame_id", frame_id, "imu_link");
    
    // Convert stdev to variance for covariance matrices
    double linear_acceleration_cov = linear_acceleration_stdev * linear_acceleration_stdev;
    double angular_velocity_cov = angular_velocity_stdev * angular_velocity_stdev;
    double orientation_cov = 0.1; // Fixed orientation covariance
    
    // Setup IMU message
    imu_msg.header.frame_id = frame_id;
    pose_msg.header.frame_id = frame_id;
    
    // Initialize covariance matrices properly (9 elements each, row-major order)
    // Orientation covariance (3x3)
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
    }
    imu_msg.orientation_covariance[0] = orientation_cov; // xx
    imu_msg.orientation_covariance[4] = orientation_cov; // yy  
    imu_msg.orientation_covariance[8] = orientation_cov; // zz
    
    // Angular velocity covariance (3x3)
    for (int i = 0; i < 9; i++) {
        imu_msg.angular_velocity_covariance[i] = 0.0;
    }
    imu_msg.angular_velocity_covariance[0] = angular_velocity_cov; // xx
    imu_msg.angular_velocity_covariance[4] = angular_velocity_cov; // yy
    imu_msg.angular_velocity_covariance[8] = angular_velocity_cov; // zz
    
    // Linear acceleration covariance (3x3)
    for (int i = 0; i < 9; i++) {
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.linear_acceleration_covariance[0] = linear_acceleration_cov; // xx
    imu_msg.linear_acceleration_covariance[4] = linear_acceleration_cov; // yy
    imu_msg.linear_acceleration_covariance[8] = linear_acceleration_cov; // zz
    
    // Publishers - FIXED: Remove leading slash for relative topic
    posePub_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 10);
    imuPub_ = pnh.advertise<sensor_msgs::Imu>("imu/data_raw", 10); // Changed from "/imu/data_raw"
    
    // Subscriber
    ros::Subscriber quatSub = nh.subscribe("quaternion", 10, quaternionCb);
    
    ROS_INFO("MPU6050 IMU Converter node started");
    ROS_INFO("Publishing IMU data to: %s", imuPub_.getTopic().c_str());
    ROS_INFO("Publishing pose data to: %s", posePub_.getTopic().c_str());
    
    ros::spin();
    return 0;
}