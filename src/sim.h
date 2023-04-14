#ifndef SIM_H
#define SIM_H

#include <vector>
#include <thread>
#include "graph.h"
#include "agent.h"

#define SIM_WIDTH 700
#define SIM_HEIGHT 700
#define SCALE 200 //1 meter / 200 pixels

extern const graph* roadmap;
extern const std::vector<obstacle>* obss;
extern std::thread sim_thread;

void init_sim(graph* g, std::vector<obstacle>* obss);
std::thread& run_sim();

#endif