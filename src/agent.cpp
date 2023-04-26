#include "agent.h"

#include <chrono>
#include <string>
//TODO integrate with ros

std::vector<agent*> agent::agents;
std::vector<obstacle> obs;

agent::agent(int id, ros::NodeHandle& nh) : pos(_pos), vel(_vel), nh(nh), id(id) {
    agents.push_back(this);
    /*obs.push_back(obstacle( 1,  1, 0.1));
    obs.push_back(obstacle(0.5, 1, 0.1));
    obs.push_back(obstacle( 0,  1, 0.1));
    obs.push_back(obstacle(-1, -1, 0.1));*/
    
    pub = nh.advertise<geometry_msgs::Twist>(std::string("/") + std::to_string(id) + std::string("/vel_commander"), 50);
    //sub = nh.subscribe(std::string("/") + std::to_string(id) + std::string("/position"), 20, &agent::fetch_pos, this);
}

agent::agent(int id, ros::NodeHandle& nh, vector3f posa) : pos(_pos), vel(_vel), nh(nh), id(id) {
    _pos = posa;
    agents.push_back(this);
    //obs.push_back(obstacle(1.1, 1, 0.1));
    obs.push_back(obstacle( 0.2,  1, 0.1));
    obs.push_back(obstacle( 0.6,  1, 0.1));

    pub = nh.advertise<geometry_msgs::Twist>(std::string("/") + std::to_string(id) + std::string("/vel_commander"), 50);
    //sub = nh.subscribe(std::string("/") + std::to_string(id) + std::string("/position"), 20, &agent::fetch_pos, this);
}

void agent::add_objective(vector3f p) {
    objectives.push(p);
}

void agent::clear_objectives() {
    for (int i = 0; i < objectives.size(); objectives.pop(), ++i);
}

vector3f agent::cur_objective() {
    return objectives.front();
}

void agent::start() {
    thread = std::thread(&agent::_start, this);
}

void agent::_start() {
    active = true;
    
    if (objectives.size() > 0) {
        apf(objectives.front());
        objectives.pop();
    }

    active = false;
}

void agent::stop() {
    active = false;
    while (!objectives.empty()) objectives.pop();
    thread.join();
}

void agent::pause() {

}

bool agent::completed() const {
    return !active && objectives.size() == 0 ? true : false;
}

bool agent::completed_all() {
    for (agent* a : agents) if (!a->completed()) return false;
    return true;
}

//TODO handle collisions
void agent::go_to(vector3f p) {
    vector3f err, prev_err;
    vector3f integral, derivative;
    vector3f input;
    float dt = 0.01;

    while (active && !reached(p)) { 
        vector3f err = p - pos;
        integral = integral + (err * dt);
        derivative = (err - prev_err) / dt;

        input = (err*KP) + (integral*KI) + (derivative*KD);
        prev_err = err;
        
        _pos = _pos + input*dt;
        _vel = input;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        //publish input as velocity
        
    }

    std::cout << pos.x << ", " << pos.y << std::endl;
}

void agent::go_to(vector2f p) {
    go_to(vector3f(p.x, p.y, pos.z));
}

void agent::apf(vector3f p) {
    std::cout << p.x << ", " << p.y << ", " << p.z << " DENE" << std::endl;
    vector3f repl, attr, zero;
    ros::Rate rate(750);

    while (!reached(p)) {
        repl.x = 0; repl.y = 0; repl.z = 0; attr.x = 0; attr.y = 0; attr.z = 0;
        
        for (agent* a : agents) {
            if (a == this) continue;
            if (dist3(pos, a->pos) < 0.5) {
                vector3f dir = pos - a->pos; dir = dir / dist3(dir, zero);
                repl = repl + (dir * (0.005 * (1/dist3(pos, a->pos) - 1/0.5)*1/pow(dist3(pos, a->pos),2)));
            }
        }
        
        obstacle min = obs[0]; vector3f minv(min.x, min.y, pos.z);
        for (obstacle o : obs) {
            vector3f opos(o.x, o.y, pos.z);
            if (dist2(opos, pos) < dist2(pos, minv)) {
                min = o;
                minv = vector3f(min.x, min.y, pos.z);
            }
            
        }

        float d = dist3(pos, minv) - min.radius;
        if (d < 0.5) {
            vector3f dir = _pos - minv; dir = dir / dist3(dir, zero);
            repl = repl + (dir * (0.005 * (1/d - 1/0.5)*1/pow(d,2)));
        }
        
        attr = ((p - _pos) * 0.75) / dist3(p, _pos);
        _vel = repl + attr;

        if (dist3(_vel, zero) > 0.5) _vel = (_vel / dist3(_vel, zero))*0.5;
        
        fetch_pos();

        twist_vel.linear.x = _vel.x;
        twist_vel.linear.y = _vel.y;
        twist_vel.linear.z = _vel.z;
        pub.publish(twist_vel);

        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        rate.sleep();
    }

    std::cout << "Completed " << std::endl;
    twist_vel.linear.x = 0;
    twist_vel.linear.y = 0;
    twist_vel.linear.z = 0;
    pub.publish(twist_vel);
}

/*/
void agent::go_to() {
    vector3f p = vector3f(objectives.front().x, objectives.front().y, 0);
    vector3f n = vector3f(objectives.front().x, objectives.front().y, 0);
    vector3f err, prev_err;
    vector3f integral, derivative;
    vector3f input;
    float dt = 0.01;

    while (active && !reached(p)) { 
        vector3f err = p - pos;
        integral = integral + (err * dt);
        derivative = (err - prev_err) / dt;

        input = (err*KP) + (integral*KI) + (derivative*KD);
        prev_err = err;
        
        _pos = _pos + input*dt;
        _vel = input;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        //publish input as velocity
        if (dist3(p, n) > 0.05) {
            vector3f zero, dir = n - p;
            dir = dir / dist3(dir, zero);
            dir = dir * 0.01;

            p = p + dir;
        } else if (!objectives.empty()) { objectives.pop(); n = vector3f(objectives.front().x, objectives.front().y, 0);}
    }

    std::cout << pos.x << ", " << pos.y << std::endl;
}*/

bool agent::reached(vector3f p) const {
    //std::cout << pos.x << ", " << pos.y << ", " << pos.z << " DIST3" << std::endl;
    return dist3(_pos, p) < ERROR ? true : false;
}

void agent::fetch_pos() {
    const geometry_msgs::PoseStamped::ConstPtr &msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(std::string("/") + std::to_string(id) + std::string("/position"));
    _pos.x = msg->pose.position.x;
    _pos.y = msg->pose.position.y;
    _pos.z = msg->pose.position.z;
}

agent::~agent() {
    //remove from agents list
    thread.join();
}

