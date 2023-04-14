#include <ros/ros.h>
#include <map>
#include <string>
#include "agent.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;
    
    std::map<std::string, bool> map;
    nh.getParam("/crazyflies", map);

    for (auto it = map.begin(); it != map.end(); ++it) {
        std::cout << (*it).first << ", " << (*it).second << std::endl;
    }

    agent a(6, nh, vector3f());
}