/**
 * @file iris_take_off.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

geometry_msgs::PoseStamped pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
    // ROS_INFO("Pose: x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

const char* topic_fomatter(int n, const char* rest){
    static char topic[100];
    sprintf(topic, "/uav%d/%s", n, rest);
    return topic;
}

const char* node_fomatter(int n, const char* node_name){
    static char topic[11];
    sprintf(topic, "%s%d", node_name, n);
    return topic;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_fomatter(atoi(argv[1]), "iris_takeoff"));
    ROS_INFO("offb_node started");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (topic_fomatter(atoi(argv[1]), "mavros/state"), 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (topic_fomatter(atoi(argv[1]), "mavros/cmd/arming"));
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (topic_fomatter(atoi(argv[1]), "mavros/set_mode"));
    ros::Publisher velo_pub = nh.advertise<geometry_msgs::Twist>
            (topic_fomatter(atoi(argv[1]), "mavros/setpoint_velocity/cmd_vel_unstamped"), 10);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (topic_fomatter(atoi(argv[1]), "mavros/local_position/pose"), 10, pose_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    // wait for FCU connection
    ROS_INFO("Connecting to FCU");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::Twist velo;
    velo.linear.x = 0;
    velo.linear.y = 0;
    velo.linear.z = 1;

    // //send a few setpoints before starting
    // ROS_INFO("Sending initial setpoints");
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     velo_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ROS_INFO("arming");
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        velo.linear.x = 0;
        velo.linear.y = 0;
        velo.linear.z = 2.0 - pose.pose.position.z;
        velo_pub.publish(velo);
        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
