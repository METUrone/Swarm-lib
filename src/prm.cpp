#include <iostream>
#include <vector>
#include <cmath>
#include <random>

#include "graph.h"
#include "agent.h"

#ifdef SIM
    #include <thread>
    #include "sim.h"
#endif

#define WIDTH 3.5
#define HEIGHT 3.5

#define SAMPLE_SIZE 1000
#define N_NEAR 7
#define MIN_DIST 0.20
#define SEGMENT_THRESHOLD 0.02 //for detecting edge intersection

#define INT_MAX 0x7FFFFFFF

using std::cout, std::endl;

void connect_all();
void sample();
bool point_intersects(float x, float y);
bool edge_intersects(float x1, float y1, float x2, float y2);
void sort_nearest(node** arr, node* n);
inline float dist(float x1, float y1, float x2, float y2);
void init_obstacles();

std::vector<obstacle> obstacles;
graph g;

void sample() {
    std::uniform_real_distribution<float> unif_horz(-WIDTH / 2, WIDTH / 2);
    std::uniform_real_distribution<float> unif_vert(-HEIGHT / 2, HEIGHT / 2);
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < SAMPLE_SIZE; ++i) {
        float x = unif_horz(gen);
        float y = unif_vert(gen);

        if (point_intersects(x, y)) {--i; continue;}
        g.add_node(x, y);
    }

}

void connect_single(node* n) {
    std::vector<node*>::const_iterator i;

    node max(INT_MAX, INT_MAX, -1);
    node* mins[N_NEAR];

    for (int k = 0; k < N_NEAR; mins[k] = &max, ++k);

    for (i = g.begin(); i < g.end(); ++i) {
        if ((*i)->index == n->index) continue;
        
        float d = dist(n->x, n->y, (*i)->x, (*i)->y);
        if (d > MIN_DIST && d < dist(n->x, n->y, mins[N_NEAR-1]->x, mins[N_NEAR-1]->y)) {
            mins[N_NEAR-1] = *i;
            sort_nearest(mins, n);
        }
    }

    for (int k = 0; k < N_NEAR; ++k) {
        node* near = mins[k];
        if (near != &max && !edge_intersects(n->x, n->y, near->x, near->y))
            g.add_edge(n, near);
    }
}

void connect_single(int id) {
    connect_single(*(g.begin() + id));
}

void connect_all() {
    std::vector<node*>::const_iterator i;
    std::vector<node*>::const_iterator j;

    node max(INT_MAX, INT_MAX, -1);
    node* mins[N_NEAR];

    for (int k = 0; k < N_NEAR; mins[k] = &max, ++k);

    for (i = g.begin(); i < g.end(); ++i) {
        node* cur = *i;
        for (j = g.begin(); j < g.end(); ++j) {
            if (cur->index == (*j)->index) continue;

            float d = dist(cur->x, cur->y, (*j)->x, (*j)->y);
            if (d > MIN_DIST && d < dist(cur->x, cur->y, mins[N_NEAR-1]->x, mins[N_NEAR-1]->y)) {
                mins[N_NEAR-1] = *j;
                
                sort_nearest(mins, cur);
            }
        }

        for (int k = 0; k < N_NEAR; ++k) {
            node* near = mins[k];
            if (near != &max && !edge_intersects(cur->x, cur->y, near->x, near->y))
                g.add_edge(cur, near);
        }

        for (int k = 0; k < N_NEAR; mins[k] = &max, ++k);
    }
}

bool edge_intersects(float x1, float y1, float x2, float y2) {
    if (dist(x1, y1, x2, y2) < SEGMENT_THRESHOLD) return false;

    float mid_x = (x1 + x2) / 2;
    float mid_y = (y1 + y2) / 2;

    if (point_intersects(mid_x, mid_y)) return true;
    else return edge_intersects(x1, y1, mid_x, mid_y) || edge_intersects(x2, y2, mid_x, mid_y);
}

bool point_intersects(float x, float y) {
    for (obstacle o : obstacles) {
        float d = dist(x, y, o.x, o.y);
        if (d < o.radius + AGENT_RADIUS) return true;
    }

    return false;
}

void sort_nearest(node** arr, node* n) {
    for (int i = 0; i < N_NEAR; ++i) {
        for (int j = 0; j < N_NEAR - i - 1; ++j) {
            float d1 = dist(n->x, n->y, arr[j]->x, arr[j]->y);
            float d2 = dist(n->x, n->y, arr[j+1]->x, arr[j+1]->y);

            if (d1 > d2) {
                node* tmp = arr[j];
                arr[j] = arr[j+1]; 
                arr[j+1] = tmp;
            }
        }
    }
}

std::vector<vector2f> form_path(agent& a, vector2f goal) {
    node* start = g.add_node(a.pos.x, a.pos.y);
    node* end = g.add_node(goal.x, goal.y);

    connect_single(start); connect_single(end);
    
    std::vector<node*> path_n = g.find_path(start, end);
    std::vector<vector2f> path_v; path_v.reserve(path_n.size());

    for (auto it = path_n.rbegin(); it < path_n.rend(); ++it)
        path_v.push_back(vector2f((*it)->x, (*it)->y));
    
    g.pop_node(); g.pop_node();

    return path_v;
}

void init_obstacles() {
    obstacles.push_back(obstacle( 1,  1, 0.1));
    obstacles.push_back(obstacle(0.5,  1, 0.1));
    obstacles.push_back(obstacle( 0, 1, 0.1));
    obstacles.push_back(obstacle(-1, -1, 0.1));
}

inline float dist(float x1, float y1, float x2, float y2) {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

int main(int argc, char *argv[]) {
    init_obstacles();

    sample();
    connect_all();

    agent a(vector3f(0.f, 0, 0)), b(vector3f(0.5f, 0, 0)), c(vector3f(-0.5f, 0, 0));

    std::vector<vector2f> path = form_path(a, vector2f(0.5f, 1.7f));
    std::vector<vector2f> path_b = form_path(b, vector2f(1.f, 1.7f));
    std::vector<vector2f> path_c = form_path(c, vector2f(0.f, 1.7f));
    float totaldist = dist((*g.begin())->x, (*g.begin())->y, path[path.size() - 1].x, path[path.size() - 1].y);
    cout << "Printing shortest path from 0 to " << SAMPLE_SIZE - 1 << endl;
    cout << "0 -> ";
    for (int i = 0; i < path.size(); ++i) {
        cout << "(" << path[i].x << "," << path[i].y << ") -> ";
        totaldist += dist(path[i].x, path[i].y, path[i-1].x, path[i-1].y);
    }
    cout <<"(" << path[path.size() - 1].x << "," << path[path.size() - 1].y << ")\n";
    cout << "Milestone count: " << path.size() << endl; 
    cout << "Distance with roadmaps: " << totaldist << endl;
    cout << "Direct distance: " << dist((*g.begin())->x, (*g.begin())->y, (*(--g.end()))->x, (*(--g.end()))->y) << endl;
    cout << "-----------------------\n";
    
    #ifdef SIM
        init_sim(&g, &obstacles);
        std::thread& sim_thread = run_sim();

        for (vector2f v : path) a.add_objective(vector2f(v.x, v.y));
        for (vector2f v : path_b) b.add_objective(vector2f(v.x, v.y));
        for (vector2f v : path_c) c.add_objective(vector2f(v.x, v.y));
        
        a.start();
        b.start();
        c.start();

        sim_thread.join();
    #endif

    return 0;
}
