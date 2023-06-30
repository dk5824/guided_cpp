#include <unistd.h>
#include <ros/ros.h>
#include "guided_cpp/uav.hpp"


Uav::Uav(){

    //subscriber
    state_sub = nh.subscribe("/mavros/state", 100, &Uav::stateCallback,this);
    local_sub = nh.subscribe("/mavros/local_position/pose", 100, &Uav::localPositionCallback,this);    
    vel_sub = nh.subscribe("/mavros/local_position/velocity_body", 100, &Uav::velocityCallback,this);
    global_sub = nh.subscribe("/mavros/global_position/raw/fix", 100, &Uav::globalPositionCallback,this);

    local_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    global_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 100);
    vel_stamped_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);
}

Uav::~Uav(){}

//// - callback functions - ////

void Uav::stateCallback(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
    // ROS_INFO_STREAM(current_state);
}

void Uav::localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    current_local_position = *msg;
    // ROS_INFO("x : %.2f | y : %.2f | z : %.2f\n" , current_local_position.pose.position.x ,
    // current_local_position.pose.position.y ,current_local_position.pose.position.z );
}

void Uav::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    current_velocity = *msg;
    // ROS_INFO_STREAM(current_velocity);
}

void Uav::globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_global_position = *msg;
    // ROS_INFO("current lat: %f", current_global_position.latitude); //for debug
}




//// - Functions - ////

void Uav::initialize() { //get parameters and set variables, frame_id, etc...

    // Get Parameter  
    nh.getParam("/guided_cpp/takeoff_height", takeoff_height);
    nh.getParam("/guided_cpp/velocity/cruise", cruising_velocity);


    //set frame_id about body_frame message
    vel_stamped.header.frame_id = "base_link"; //essential?    
}

void Uav::arming() {
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    arm_cmd.request.value = true;
    
    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }
}

void Uav::takeoff(int height){

    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");

    ros::Rate loop_rate(10); //10Hz

    //takeoff(also land) service requires 5 float32 variables
    takeoff_cmd.request.min_pitch = 0;
    takeoff_cmd.request.yaw = 0;
    takeoff_cmd.request.latitude = 0;
    takeoff_cmd.request.longitude = 0;
    takeoff_cmd.request.altitude = height;
    
    if(takeoff_client.call(takeoff_cmd)){
        std::cout << int(takeoff_cmd.response.success) << std::endl;
        ROS_INFO("tring to takeoff");
        while(abs(current_local_position.pose.position.z-height)>=0.1){
            ROS_INFO("current_height: %f", current_local_position.pose.position.z);
            ros::spinOnce();
            loop_rate.sleep();
        }
        ROS_INFO("finish takeoff");
    }
}

void Uav::land() {

    /*
    landing by changing flight mode to LAND

    mode_cmd.request.custom_mode = "LAND";
    if( set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent){
        ROS_INFO("LAND enabled");
    }
    */

    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    //takeoff(also land) service requires 5 float32 variables
    land_cmd.request.min_pitch = 0;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    if(land_client.call(land_cmd)){
        std::cout << int(land_cmd.response.success) << std::endl;
        ROS_INFO("trying to land");
        while(current_local_position.pose.position.z>0){
            ros::spinOnce();
        }
        ROS_INFO("finish land");
    }

}

void Uav::switch_modes(std::string next_mode) {
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mode_cmd.request.base_mode = 0;
    mode_cmd.request.custom_mode = next_mode;

    if( set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO_STREAM("successfully changed to " << next_mode << " mode \n");
    }

}

void Uav::go_by_local(int x, int y, int z) { //  flying to (x,y,z) point in terms of local coordinate

    ROS_INFO("go to position x: %d  y: %d  z: %d", x, y, z);
    
    local_setpoint.header.stamp = ros::Time::now();
    
    ros::Rate func_rate(10); // 2Hz

    
    local_setpoint.header.frame_id = "base_link";
    
    local_setpoint.pose.position.x = x;
    local_setpoint.pose.position.y = y;
    local_setpoint.pose.position.z = z;

    local_setpoint.pose.orientation = current_local_position.pose.orientation;
    
    while(abs(current_local_position.pose.position.x-x)>=0.1 ||
        abs(current_local_position.pose.position.y-y)>=0.1 ||
        abs(current_local_position.pose.position.z-z)>=0.1) {    
        
        ROS_INFO("x: %f   ||  y: %f  ||  z: %f", current_local_position.pose.position.x, 
        current_local_position.pose.position.y, current_local_position.pose.position.z);
        
        local_pub.publish(local_setpoint);
        ros::spinOnce(); //This cmd is Essential because we need to subscribe current_local_position topic!!
        
        func_rate.sleep();
    }

    ROS_INFO("reached to the setpoint x: %d  y: %d  z: %d", x, y, z);
}

void Uav::go_by_global(double lat, double lon, double alt) { // flying to certain latitude, longitude, altitude 

    ros::Rate func_rate(10);

    ROS_INFO("go to position lat: %f  lon: %f  alt: %f", lat, lon, alt);


    global_setpoint.header.stamp = ros::Time::now();
    global_setpoint.header.frame_id = "map";

    global_setpoint.pose.position.latitude = lat;
    global_setpoint.pose.position.longitude = lon;
    global_setpoint.pose.position.altitude = alt;


    global_setpoint.pose.orientation = current_local_position.pose.orientation;
    
    while(abs(current_global_position.latitude-lat)>=0.1 ||
        abs(current_global_position.longitude-lon)>=0.1 ||
        abs(current_global_position.altitude-alt)>=0.1) {    
        
        ROS_INFO("x: %f   ||  y: %f  ||  z: %f", current_global_position.latitude, 
        current_global_position.longitude, current_global_position.altitude);
        
        global_pub.publish(global_setpoint);
        ros::spinOnce(); //This cmd is Essential because we need to subscribe current_global_position topic!!
        
        func_rate.sleep();
    }

    ROS_INFO("reached to the setpoint lat: %f  lon: %f  alt: %f", lat, lon, alt);    
}


void Uav::setLinearVelocity(double x, double y, double z) {
    vel_stamped.twist.linear.x = x;
    vel_stamped.twist.linear.y = y;
    vel_stamped.twist.linear.z = z;

    vel_stamped.header.frame_id = "base_link";
    vel_stamped.header.stamp = ros::Time::now();

    vel_stamped_pub.publish(vel_stamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_controller");
    Uav Uav;

    ros::Rate loop_rate(20);

    Uav.initialize();
    Uav.switch_modes("GUIDED");
    Uav.arming();
    Uav.takeoff(Uav.takeoff_height);
    sleep(2);

    Uav.go_by_local(2,2,4);
    
    while(Uav.timecnt <= 10) {
        Uav.setLinearVelocity(0, Uav.cruising_velocity, 0);
        ros::spinOnce();
        Uav.timecnt += 0.05; // 1/20(Period)
        loop_rate.sleep();
    }

    Uav.land();

    return 0;
}