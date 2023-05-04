#ifndef RRT_H
#define RRT_H

#include "obstacles.h"
#include <stdlib.h>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
    float cost;
    int id;
    // uint childcount;
    // ushort[9] children;
    //assumption is that we only have 4 children at most;
    //children list;
    // is float good? Can we make it better?
    // is short enough?
};

class RRT
{
public:
    RRT();
    void initialize();
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    float distance(Vector2f &p, Vector2f &q);
    Vector2f newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    void update(Node *q);
    void update_value(Node * cc);
    bool reached();
    float get_cost(Node* q, Node *qNearest);
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void deleteNodes(Node *root, int call);
    Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2f startPos, endPos;
    int max_iter;
    int step_size;
};

#endif // RRT_H
