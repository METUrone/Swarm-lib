#include <SFML/Graphics.hpp>
#include <iostream>
#include <list>
#include "sim.h"

const graph* roadmap;
const std::vector<obstacle>* obss;
std::thread sim_thread;

std::vector<agent*>& agents = agent::agents;
std::vector<sf::CircleShape> milestones;
std::vector<sf::RectangleShape> agshapes;
std::vector<sf::CircleShape> obshapes;
std::vector<sf::Vertex*> edges;

sf::RenderWindow window(sf::VideoMode(SIM_WIDTH, SIM_HEIGHT), "PRM DEMO");
sf::Color dgray(0x606060);
sf::Color lgray(0xD0D0D0FF);

sf::Vector2<float> transform(vector3f v, float offset=0) {
    sf::Vector2<float> u(v.x, v.y);
    u.x += offset; u.y += offset;
    u.x *= SCALE; u.y *= SCALE; u.y = SIM_HEIGHT - u.y;
    u.x += SIM_WIDTH / 2; u.y -= SIM_HEIGHT / 2;
    return u;
}

void init_shapes() {
    for (auto it = roadmap->begin(); it < roadmap->end(); ++it) {
        sf::CircleShape c;
        c.setPosition(transform(vector3f((*it)->x, (*it)->y, 0)));
        c.setFillColor(sf::Color::Red);
        c.setRadius(2);
        milestones.push_back(c);
    }
    auto ite = roadmap->egde_begin();
    auto itn = roadmap->begin();
    
    for (; itn < roadmap->end(); ++itn, ++ite) {
        std::list<node*> neigs = *ite;
        node* cur = *itn;

        for (node* n : neigs) {
            sf::Vertex* vs = new sf::Vertex[2];
            vs[0].position = transform(vector3f(cur->x, cur->y, 0));
            vs[0].color = dgray;
            vs[1].position = transform(vector3f(n->x, n->y, 0));
            vs[1].color = dgray;
            edges.push_back(vs);
        }
    }

    for (auto it = obss->cbegin(); it < obss->cend(); ++it) {
        obstacle o = (*it);
        sf::CircleShape c;
        c.setPosition(transform(vector3f(o.x, o.y, 0)));
        c.setRadius(o.radius * SCALE);
        c.setOrigin(c.getRadius(), c.getRadius());
        c.setFillColor(lgray);
        obshapes.push_back(c);
    }

    for (auto it = agents.begin(); it < agents.end(); ++it) {
        sf::RectangleShape r;
        r.setFillColor(sf::Color::Blue);
        r.setSize(sf::Vector2f(10, 10));
        agshapes.push_back(r);
    }
}

void init_sim(graph* _roadmap, std::vector<obstacle>* _obss) {
    roadmap = _roadmap;
    obss = _obss;

    init_shapes();
}

void _run_sim() {
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        for (int i = 0; i < agshapes.size(); ++i)
            agshapes[i].setPosition(transform(agents[i]->pos));

        window.clear();
        for (sf::CircleShape c : obshapes) window.draw(c);
        //for (sf::Vertex* line : edges) window.draw(line, 2, sf::Lines);
        //for (sf::CircleShape c : milestones) window.draw(c);
        for (sf::RectangleShape r : agshapes) window.draw(r);
        window.display();
    }

    for (sf::Vertex* vs : edges) delete[] vs;
}

std::thread& run_sim() {
    sim_thread = std::thread(_run_sim);
    return sim_thread;
}

