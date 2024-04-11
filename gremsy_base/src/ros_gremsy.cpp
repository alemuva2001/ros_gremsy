#include <gremsy_base/ros_gremsy.h>

#define DEGREES


GimbalNode::GimbalNode(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // Initialize dynamic-reconfigure
    dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig> server(pnh);
    dynamic_reconfigure::Server<gremsy_base::ROSGremsyConfig>::CallbackType f;
    f = boost::bind(&GimbalNode::reconfigureCallback, this, _1, _2);
    server.setCallback(f);

    // Init state variables
    goals_.vector.x = 0; goals_.vector.y = 0; goals_.vector.z = 0;
    offset_x = 0; offset_y = 0; offset_z = 0;

    offs = goals_;

    // Advertive Publishers
    encoder_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ros_gremsy/encoder", 1000);
    angle_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ros_gremsy/angle", 1000);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/ros_gremsy/imu/data", 10);

    gimbal_goal_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/ros_gremsy/goals", 1);

    // Register Subscribers
    gimbal_goal_sub = nh.subscribe("/ros_gremsy/goals", 1, &GimbalNode::setGoalsCallback, this);
    
    gimbal_goal_srv = nh.advertiseService("/ros_gremsy/goal", &GimbalNode::setGoalSrvCallback, this);
    gimbal_mode_srv = nh.advertiseService("/ros_gremsy/mode", &GimbalNode::setModeSrvCallback, this);

    // Define SDK objects
    serial_port_ = new Serial_Port(config_.device.c_str(), config_.baudrate);
    gimbal_interface_ = new Gimbal_Interface(serial_port_, 1, MAV_COMP_ID_ONBOARD_COMPUTER, Gimbal_Interface::MAVLINK_GIMBAL_V1);

    // Start ther serial interface and the gimbal SDK
    serial_port_->start();
    gimbal_interface_->start();

    // Wait for gimbal present
    while (!gimbal_interface_->present()) {
        ros::Duration(0.2).sleep();
    }

    ///////////////////
    // Config Gimbal //
    ///////////////////

    // Check if gimbal is on
    if(gimbal_interface_->get_gimbal_status().mode == Gimbal_Interface::GIMBAL_STATE_OFF)
    {
        // Turn on gimbal
        ROS_INFO("TURN_ON!\n");
        gimbal_interface_->set_gimbal_motor(Gimbal_Interface::TURN_ON);
    }

    // Wait until the gimbal is on
    while (gimbal_interface_->get_gimbal_status().mode < Gimbal_Interface::GIMBAL_STATE_ON)
    {
        ROS_WARN("Waiting for the gimbal to turn on!\n");
        ros::Duration(0.2).sleep();
    }

    ros::Timer poll_timer = nh.createTimer(
        ros::Duration(1/config_.state_poll_rate),
        &GimbalNode::gimbalStateTimerCallback, this);

    ros::Timer goal_timer = nh.createTimer(
        ros::Duration(1/config_.goal_push_rate),
        &GimbalNode::gimbalGoalTimerCallback, this);

    gimbal_interface_->set_gimbal_lock_mode_sync();

    // Configure the gimbal to send angles as encoder values
    if(config_.encoder_type == 1){
        gimbal_interface_->set_gimbal_encoder_type_send(false);
        ROS_INFO("Gimbal encoder provides angle\n");
    }
    if(config_.encoder_type == 0){
        gimbal_interface_->set_gimbal_encoder_type_send(true);
        ROS_INFO("Gimbal encoder provides encoder values\n");
    }
    
    ros::spin();
}


void GimbalNode::gimbalStateTimerCallback(const ros::TimerEvent& event)
{
    // Publish Gimbal Encoder Values
    attitude<int16_t> encoder_values = gimbal_interface_->get_gimbal_encoder();

    geometry_msgs::Vector3Stamped encoder_ros_msg;

    //if (mode > 5) ros::Duration(3).sleep();

    encoder_ros_msg.header.stamp = ros::Time::now();
    encoder_ros_msg.vector.x = ((float) encoder_values.roll); //* DEG_TO_RAD;
    encoder_ros_msg.vector.y = ((float) encoder_values.pitch); //* DEG_TO_RAD;
    encoder_ros_msg.vector.z = ((float) encoder_values.yaw); //* DEG_TO_RAD;


    //Convert the motor encoder values to angle using linear interpolation
    angle_ros_msg.header.stamp = ros::Time::now();
    //Eje Z
    angle_ros_msg.vector.z = ((float) encoder_values.yaw + 17570)/182;
    //if (encoder_ros_msg.vector.z > 180) encoder_ros_msg.vector.z -= 360;

    //Eje Y
    angle_ros_msg.vector.y = ((float) encoder_values.pitch + 29290)/181;
    if (encoder_ros_msg.vector.y > 180) encoder_ros_msg.vector.y -= 363;

    //Eje X
    angle_ros_msg.vector.x = ((float) encoder_values.roll + 5588)/182;

    if(config_.encoder_type == 1) angle_ros_msg = encoder_ros_msg;

    // Initialize gimbal 
    if (mode > 5) GimbalNode::initialization();
    //gimbal_interface_->set_gimbal_lock_mode_sync();


    if (encoder_values.roll or encoder_values.pitch or encoder_values.yaw)
    {
        encoder_pub.publish(encoder_ros_msg);
        angle_pub.publish(angle_ros_msg);


        ROS_INFO("diff: x: %f\t y: %f\t z:%f\n",offset_x + goals_.vector.x - angle_ros_msg.vector.x, offset_y + goals_.vector.y - angle_ros_msg.vector.y, offset_z + goals_.vector.z - angle_ros_msg.vector.z);
        ROS_INFO("OFFSET: x: %f\t y: %f\t z: %f\n", offset_x, offset_y, offset_z);
    }

    //// Publish Gimbal IMU (Currently this deadlocks)

    // Create ROS message
    sensor_msgs::Imu imu_message;

    // Set header
    imu_message.header.stamp = encoder_ros_msg.header.stamp;

    //// Publish Camera Mount Orientation
    attitude<float> processed_gimbal_attitude = gimbal_interface_->get_gimbal_attitude();

    Eigen::Quaterniond imu_quaternion = convertYXZtoQuaternion(
        processed_gimbal_attitude.roll,
        -processed_gimbal_attitude.pitch,
        processed_gimbal_attitude.yaw);

    imu_message.orientation.x = imu_quaternion.x();
    imu_message.orientation.y = imu_quaternion.y();
    imu_message.orientation.z = imu_quaternion.z();
    imu_message.orientation.w = imu_quaternion.w();

    imu_pub.publish(imu_message);

    geometry_msgs::TransformStamped transform_imu;
    transform_imu.header.stamp = imu_message.header.stamp;
    transform_imu.header.frame_id = "imu_frame";
    transform_imu.child_frame_id = "map";
    transform_imu.transform.rotation = tf2::toMsg(imu_quaternion);
    bc_.sendTransform(transform_imu);
}


void GimbalNode::gimbalGoalTimerCallback(const ros::TimerEvent& event)
{
    gimbal_interface_->set_gimbal_rotation_sync(
        -goals_.vector.y,    //* RAD_TO_DEG
        goals_.vector.x,     //* RAD_TO_DEG
        goals_.vector.z);    //* RAD_TO_DEG

}


bool GimbalNode::setGoalSrvCallback(gremsy_base::GimbalPos::Request& req, gremsy_base::GimbalPos::Response& res)
{
    if (mode == 0){
    offset_x = angle_ros_msg.vector.x - goals_.vector.x;
    offset_y = angle_ros_msg.vector.y - goals_.vector.y;
    offset_z = angle_ros_msg.vector.z - goals_.vector.z;
    }

    goals_.header.stamp = ros::Time::now();
    goals_.vector.x = req.pos.x; goals_.vector.y = req.pos.y; goals_.vector.z = req.pos.z;

    //Store the current location to mode change
    offs.vector.x = req.pos.x; offs.vector.y = req.pos.y; offs.vector.z = req.pos.z;

    //Change position taking into account the offset
    if (mode == 0){

        goals_.vector.x = goals_.vector.x - offset_x;
        goals_.vector.y = goals_.vector.y - offset_y;
        goals_.vector.z = goals_.vector.z - offset_z;

    }

    ROS_INFO("Goal: x: %f\t y: %f\t z: %f", goals_.vector.x, goals_.vector.y, goals_.vector.z);
    ROS_INFO("OFFSET: x: %f\t y: %f\t z: %f\n", offset_x, offset_y, offset_z);

    res.success = true;

    return true;
}

void GimbalNode::setGoalsCallback(geometry_msgs::Vector3Stamped message)
{

    goals_ = message;

    //Store the current location to mode change
    offs = message;

     if (mode == 0){

         goals_.vector.x = goals_.vector.x - offset_x;
         goals_.vector.y = goals_.vector.y - offset_y;
         goals_.vector.z = goals_.vector.z - offset_z;

    }

    ROS_INFO("Goal: x: %f\t y: %f\t z: %f", goals_.vector.x, goals_.vector.y, goals_.vector.z);
    ROS_INFO("OFFSET: x: %f\t y: %f\t z: %f\n", offset_x, offset_y, offset_z);
}

//Cambio el modo del Gimbal
bool GimbalNode::setModeSrvCallback(gremsy_base::GimbalMode::Request& req, gremsy_base::GimbalMode::Response& res)
{

    ROS_INFO("Changing mode to %d", req.mode);
    
    if(req.mode == 1){

        offs = angle_ros_msg;

        mode = 1;

        gimbal_interface_->set_gimbal_follow_mode_sync();
  
    } else {

        mode = 0;

        gimbal_interface_->set_gimbal_lock_mode_sync(); 

    }

    ROS_INFO("offs: x: %f\t y: %f\t z:%f\n", offs.vector.x, offs.vector.y, offs.vector.z);
    
    //It is needed to publish the location when the mode changes
    gimbal_goal_pub.publish(offs);

    res.success = true;

    return true;
}


void GimbalNode::reconfigureCallback(gremsy_base::ROSGremsyConfig &config, uint32_t level) {
    config_ = config;
}


Eigen::Quaterniond GimbalNode::convertYXZtoQuaternion(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond quat_abs(
                  Eigen::AngleAxisd(-DEG_TO_RAD * pitch, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(-DEG_TO_RAD * roll, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(DEG_TO_RAD * yaw, Eigen::Vector3d::UnitZ()));
    return quat_abs;
}


//The first time you turn on the gimbal, it is needed to get the offset for lock mode
void GimbalNode::initialization(){

    gimbal_interface_->set_gimbal_lock_mode_sync();

    ros::Duration(2).sleep();

    offset_x = angle_ros_msg.vector.x;
    offset_y = angle_ros_msg.vector.y;
    offset_z = angle_ros_msg.vector.z;

    ROS_INFO("x: %f\t y: %f\t z: %f", offset_x, offset_y, offset_z);

    goals_.vector.x = -offset_x;
    goals_.vector.y = -offset_y;
    goals_.vector.z = -offset_z;

    ros::Duration(3).sleep();

    gimbal_interface_->set_gimbal_follow_mode_sync();

    gimbal_goal_pub.publish(offs);

    mode = 1;

    ROS_INFO("Gimbal initiated in follow mode\n");
}

int main(int argc, char **argv)
{
    // Init
    ros::init(argc, argv, "gremsy_base");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    GimbalNode n(nh, pnh);

    return 0;
}
