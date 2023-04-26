#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

#include "agent.h"

using std::cout, std::endl;

std::vector<agent*> agent_list;
std::vector<obstacle> obs_list;

void read_yaml(ros::NodeHandle& nh) {
    YAML::Node yaml_file = YAML::LoadFile("/home/aeren/havelsan-swarm/src/Swarm-lib/config/crazyflies.yaml");
    YAML::Node obs_file = YAML::LoadFile("/home/aeren/havelsan-swarm/src/Swarm-lib/config/obstacles.yaml");
    YAML::Node crazyflies = yaml_file["crazyflies"];
    YAML::Node poses = obs_file["obstacles"];

    for (auto it = crazyflies.begin(); it != crazyflies.end(); ++it) {
        const YAML::Node& initial_position_node = (*it)["initialPosition"];
        float x = initial_position_node[0].as<double>(); ROS_INFO("X\n");
        float y = initial_position_node[1].as<double>(); ROS_INFO("X\n");
        float z = initial_position_node[2].as<double>();
        
        int id = (*it)["id"].as<int>();

        agent_list.push_back(new agent(id, nh, vector3f(x,y,z)));
    }

    float radius = obs_file["radius"].as<double>(); ROS_INFO("RADIUS\n");

    for (auto pos : poses) {
        obs_list.push_back(obstacle(pos["x"].as<double>(), pos["y"].as<double>(), radius)); ROS_INFO("OX, OY\n");
    }
}

void displace(vector3f shift) {
    for (agent *a : agent_list) {
        a->fetch_pos();
        a->add_objective(a->pos + shift); cout << a->pos.x << ", " << a->pos.y << ", " << a->pos.z << endl;
        a->start();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apf");
    ros::NodeHandle nh;
    
    read_yaml(nh);

    //agent::obs = obs_list;
    // for (obstacle& o : agent::obs) {
    //     cout << o.y << endl;
    // }
    ros::Duration(3).sleep();
    displace(vector3f(0.0, 2.0, 0.0));

    ros::spin();
}