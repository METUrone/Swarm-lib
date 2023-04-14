#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <iterator>
#include <vector>
#include <list>
#include <queue>
#include <cmath>

struct node {
    float x, y;
    const int index;

    node(float x, float y, int index) : index(index), x(x), y(y) {}
};

class graph {
    public:
        graph();
        graph(const graph& g);

        node* add_node(float x, float y);
        void add_edge(node* n1, node* n2);
        void pop_node();
        void clear();
        int size() const;
        std::vector<node*> find_path(node* from, node* to) const;
        std::vector<node*> find_path(int from, int to) const;
        std::vector<node*>::const_iterator begin() const;
        std::vector<node*>::const_iterator end() const;
        std::vector<std::list<node*>>::const_iterator egde_begin() const;
        std::vector<std::list<node*>>::const_iterator egde_end() const;

        //TODO overload assignment operator =

        ~graph();

    private:
        std::vector<node*> nodes;
        std::vector<std::list<node*>> edges;
};

#endif