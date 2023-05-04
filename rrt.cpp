#include "rrt.h"
int RANDOM = 0;
float CONTROL = 14.0;


RRT::RRT()
{
    obstacles = new Obstacles;
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->id = 0;
    lastNode = root;
    nodes.push_back(root);
    step_size = 3;
    max_iter = 3000;
    //here we add the cost.
}

void RRT::update_value(Node *cc)
{   
    if(cc->cost == 0){
        return;
    }
    float c = cc->parent->cost;
    cc->cost = get_cost(cc->parent, cc) + c;
    if(cc->children.size() == 0){
        return;
    }
    for(auto x: cc->children){
        update_value(x);
    }
}
/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize()
{
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->cost = 0;
    root->id = 0;
    lastNode = root;
    //only one array here
    nodes.push_back(root);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRT::getRandomNode()
{
    Node* ret;
    Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
    if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT) {
        ret = new Node;
        ret->position = point;
        return ret;
    }
    return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
float RRT::distance(Vector2f &p, Vector2f &q)
{
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */


Node* RRT::nearest(Vector2f point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

void RRT::update(Node *q)
{
    //first get each of them, then write as a full array, then mpi send
    //we could do it by adding an id field. 
    std::vector<Node *> myneighbors;
    //1. get my neighbors
    Vector2f point = q->position;
    float neighborDist = CONTROL;
    float curr_cost = q->cost;
    //cost of tentative q-parent-root.
    Node *bestneighbor = NULL;
    float cbestnei = curr_cost;
    //
    for(int i = 0; i < (int)nodes.size(); i++) {
        if(nodes[i]->id != q->id && nodes[i]->id != 0){
            float dist = distance(point, nodes[i]->position);
            if (dist < neighborDist) {
                myneighbors.push_back(nodes[i]);
                //check if my 
                float nodei_cost = nodes[i]->cost + dist;
                if(cbestnei > nodei_cost){
                    bestneighbor = nodes[i];
                    cbestnei = nodei_cost;
                }
            }
        }
    }
    if(bestneighbor != NULL){
        q->parent = bestneighbor;
        q->cost = cbestnei;
        add(bestneighbor,q);
        for(auto child: q->children){
            update_value(child);
        }
        //I should be done right?
        for(auto x: myneighbors){
            if(x->cost>0 && cbestnei + distance(x->position, q->position) < x->cost ){
                // printf("here IN?\n");
                if(x->parent->id != q->id){
                    remove(x->parent->children.begin(), x->parent->children.end(),x);
                    // printf("here OUT?\n");
                    x->parent = q;
                    for(auto k: q->children){
                        if (k->id == x->id){
                            printf("We found automyneighbor wrong -- doubled added to someone's children\n");
                        }
                    }
                    q->children.push_back(x);
                }
                x->cost = cbestnei + distance(x->position, q->position);
                update_value(x);
            }
        }
    }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Vector2f RRT::newConfig(Node *q, Node *qNearest)
{   
    // could add a bit randomnes into it

    Vector2f to = q->position;
    Vector2f from = qNearest->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRT::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    //then we have to check if the other nodes are closer to it then add the node to be parent to this node. 
    //so a update function.
    int check = 0;
    for(auto k: qNearest->children){
        if (k->id == qNew->id){
            printf("We found add wrong -- doubled added to someone's children\n");
        }
    }
    qNearest->children.push_back(qNew);
    // qNew->id = nodes.size();
    nodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRT::reached()
{
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)
        return true;
    return false;
}

void RRT::setStepSize(int step)
{
    step_size = step;
}

void RRT::setMaxIterations(int iter)
{
    max_iter = iter;
}

//can be changed later.
float RRT::get_cost(Node *q, Node *qNearest)
{
    return RRT::distance(q->position, qNearest->position);
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRT::deleteNodes(Node *root, int call)
{
    //printf("curr level: %d, node's id: %d\n",call, root->id);
    //if (root->parent != NULL)
        //printf("curr parent: %d ---------------------(%f,%f)\n",root->parent->id, root->parent->position.x(), root->parent->position.y());
    //printf("root: %d --------------------------------(%f,%f)\n",root->id, root->position.x(), root->position.y());
    if(root == NULL){
        return;
    }
    //printf("children ids:\n");
    std::vector<int> v;
    for(int i = 0; i < (int)root->children.size(); i++) {
        // printf("%d, ",root->children[i]->id);
        // if(std::find(v.begin(), v.end(), root->children[i]->id) != v.end()){
        //     printf("we found a duplicate in curr node's children id\n");
        //     v.push_back(root->children[i]->id);
        //     printf("[ ");
        //     for (int j: v)
        //         printf("%d ,", j);
        //     printf("]\n");
        // }else{
        //     v.push_back(root->children[i]->id);
        // }
        deleteNodes(root->children[i], call + 1);
        // if(root->children[i]!=NULL){
        //     deleteNodes(root->children[i], call + 1);
        // }
    }
    //printf("\n");
    //delete root;
}
