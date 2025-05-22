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
        ros::Time sensor_data_time = ros::Time::now(); // Removed time offset for reliability

        if (imu_subscribed)
        {
            // Fill IMU message
            imu_msg.header.stamp = sensor_data_time;
            imu_msg.orientation = *msg;

            // Required fields for Madgwick filter
            imu_msg.angular_velocity.x = 0.0;  // Required - set to 0 if no gyro data
            imu_msg.angular_velocity.y = 0.0;
            imu_msg.angular_velocity.z = 0.0;

            imu_msg.linear_acceleration.x = 0.0;  // Recommended
            imu_msg.linear_acceleration.y = 0.0;
            imu_msg.linear_acceleration.z = 9.81; // Default gravity value

            imuPub_.publish(imu_msg);
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

    // Setup IMU message
    imu_msg.header.frame_id = frame_id;
    pose_msg.header.frame_id = frame_id;

    // Orientation covariance (3x3 row-major)
    imu_msg.orientation_covariance = {
        0.1, 0.0, 0.0,
        0.0, 0.1, 0.0,
        0.0, 0.0, 0.1
    };

    // Angular velocity covariance (3x3 row-major)
    imu_msg.angular_velocity_covariance = {
        angular_velocity_cov, 0.0, 0.0,
        0.0, angular_velocity_cov, 0.0,
        0.0, 0.0, angular_velocity_cov
    };

    // Linear acceleration covariance (3x3 row-major)
    imu_msg.linear_acceleration_covariance = {
        linear_acceleration_cov, 0.0, 0.0,
        0.0, linear_acceleration_cov, 0.0,
        0.0, 0.0, linear_acceleration_cov
    };

    // Publishers
    posePub_ = pnh.advertise<geometry_msgs::PoseStamped>("pose", 10);
    imuPub_ = pnh.advertise<sensor_msgs::Imu>("/imu/data_raw", 10); // Relative topic name

    // Subscriber
    ros::Subscriber quatSub = nh.subscribe("quaternion", 10, quaternionCb);

    ROS_INFO("MPU6050 IMU Converter node started");
    ros::spin();

    return 0;
}