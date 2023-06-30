#include <unistd.h>
#include <ros/ros.h>


#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <geographic_msgs/GeoPoseStamped.h>

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <math.h>


class Uav {
protected:


    ros::NodeHandle nh; //Node Handler

    //subscriber
    ros::Subscriber state_sub; 
    ros::Subscriber local_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber control_sub;
    ros::Subscriber avoid_sub;
    ros::Subscriber global_sub;

    //publisher
    ros::Publisher vel_stamped_pub; //for TwistStamped Topic
    ros::Publisher local_pub;
    ros::Publisher global_pub;

    //service_client
    ros::ServiceClient arming_client;
    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    ros::ServiceClient set_mode_client;

    //arming, mode change, takeoff service
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandTOL takeoff_cmd;
    mavros_msgs::CommandTOL land_cmd;
    mavros_msgs::SetMode mode_cmd;

    //message to publish(local, global, velocity vector)
    geometry_msgs::TwistStamped vel_stamped;
    geometry_msgs::PoseStamped local_setpoint;
    geographic_msgs::GeoPoseStamped global_setpoint;


public:

    Uav();
    ~Uav();

    //message to subscribe
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_local_position;
    geometry_msgs::TwistStamped current_velocity;
    sensor_msgs::NavSatFix current_global_position;

    const double rad_to_deg = 57.29577951; // declare constant about unit conversion

    double takeoff_height, cruising_velocity;
    double timecnt = 0.0;

    // Callback Functions
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    // Extra Functions
    void initialize();
    void arming();
    void takeoff(int height);
    void land();
    void switch_modes(std::string next_mode);

    void go_by_local(int x, int y, int z);
    void go_by_global(double lat, double lon, double alt);
    void setLinearVelocity(double x, double y, double z);

};