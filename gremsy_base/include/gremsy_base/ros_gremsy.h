#include <boost/bind.hpp>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <gremsy_base/GimbalMode.h>
#include <gremsy_base/GimbalPos.h>

#include <gremsy_base/ROSGremsyConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <unistd.h>
#include "gimbal_interface.h"
#include "serial_port.h"
#include <signal.h>
#include <time.h>
#include <iostream>

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

class GimbalNode
{
public:
    // Params: (public node handler (for e.g. callbacks), private node handle (for e.g. dynamic reconfigure))
    GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh);
private:
    // Dynamic reconfigure callback
    void reconfigureCallback(gremsy_base::ROSGremsyConfig &config, uint32_t level);
    // Timer which checks for new infomation regarding the gimbal
    void gimbalStateTimerCallback(const ros::TimerEvent& event);
    // Timer which sets the gimbal goals
    void gimbalGoalTimerCallback(const ros::TimerEvent& event);
    
    // Callback to set a new gimbal position
    void setGoalsCallback(geometry_msgs::Vector3Stamped message);
    bool setGoalSrvCallback(gremsy_base::GimbalPos::Request& req, gremsy_base::GimbalPos::Response& res);

    //void setModeCallback(std_msgs::Int32 message);
    bool setModeSrvCallback(gremsy_base::GimbalMode::Request& req, gremsy_base::GimbalMode::Response& res);
    void initialization();

    // Converter
    Eigen::Quaterniond convertYXZtoQuaternion(double roll, double pitch, double yaw);

    // Gimbal SDK
    Gimbal_Interface* gimbal_interface_;
    // Serial Interface
    Serial_Port* serial_port_;
    // Current config
    gremsy_base::ROSGremsyConfig config_;
    // Publishers
    ros::Publisher encoder_pub, imu_pub, angle_pub, gimbal_goal_pub;
    // Subscribers
    ros::Subscriber gimbal_goal_sub;
    //ros::Subscriber gimbal_mode_sub;
    // Services
    ros::ServiceServer gimbal_mode_srv, gimbal_goal_srv;

    // TF Broadcasters
    tf2_ros::TransformBroadcaster bc_;

    // Value store
    geometry_msgs::Vector3Stamped goals_;
    geometry_msgs::Vector3Stamped angle_ros_msg;

    //Aux variables
    float offset_x, offset_y, offset_z; //Lock mode offset
    int mode = 10;                      //Actual mode
    geometry_msgs::Vector3Stamped offs; //Store the position when change mode


};