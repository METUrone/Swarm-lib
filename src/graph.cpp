#include "graph.h"

typedef std::pair<float, node*> qpair;

const unsigned int a = 0x7f800000;
const float inf = *(float*) &a;

graph::graph() {

}

graph::graph(const graph& g) {
    //TODO implement copy constructor
}

node* graph::add_node(float x, float y) {
    node* n = new node(x, y, nodes.size());
    nodes.push_back(n);
    edges.push_back(std::list<node*>());
    return n;
}

void graph::add_edge(node* n1, node* n2) {
    if (n1->index == n2->index) return; //self connecting nodes are not allowed

    edges[n1->index].push_front(n2);
    edges[n2->index].push_front(n1);
}

void graph::pop_node() {
    node* todel = nodes[nodes.size() - 1];
    std::list<node*> ns = edges[todel->index];
    std::list<node*>::iterator i = ns.begin();

    //remove from neighbours' adjacency list
    for (; i != ns.end(); edges[(*i)->index].remove(todel), ++i);

    delete todel;
    nodes.pop_back();
    edges.pop_back();
}

void graph::clear() {
    for (int i = 0; i < nodes.size(); delete nodes[i], ++i);
    nodes.clear();
    edges.clear();
}

int graph::size() const { 
    return nodes.size(); 
}

//returns the path in reverse order (to -> from)
std::vector<node*> graph::find_path(node* from, node* to) const {
    std::vector<float> dists(nodes.size(), inf);
    std::vector<node*> prevs(nodes.size(), nullptr);
    std::priority_queue<qpair, std::vector<qpair>, std::greater<qpair>> pq;

    dists[from->index] = 0;
    pq.push(qpair(0, from));

    while (!pq.empty()) {
        node* cur = pq.top().second; pq.pop();
        if (cur == to) break;

        std::list<node*> neigs = edges[cur->index];
        for (auto it = neigs.begin(); it != neigs.end(); ++it) {
            float newdist = dists[cur->index] + sqrt(pow(cur->x - (*it)->x, 2) + pow(cur->y - (*it)->y, 2));
            if (newdist < dists[(*it)->index]) {
                dists[(*it)->index] = newdist;
                prevs[(*it)->index] = cur;
                pq.push(qpair(newdist, *it));
            }
        }
    }

    std::vector<node*> path;
    node* cur = to;
    while (cur != from) {
        path.push_back(cur);
        cur = prevs[cur->index];
    }

    return path;
}

std::vector<node*> graph::find_path(int from, int to) const {
    return find_path(nodes[from], nodes[to]);
}

std::vector<node*>::const_iterator graph::begin() const {
    return nodes.cbegin();
}

std::vector<node*>::const_iterator graph::end() const {
    return nodes.cend();
}

std::vector<std::list<node*>>::const_iterator graph::egde_begin() const {
    return edges.cbegin();
}

std::vector<std::list<node*>>::const_iterator graph::egde_end() const {
    return edges.cend();
}

graph::~graph() { 
    clear(); 
}