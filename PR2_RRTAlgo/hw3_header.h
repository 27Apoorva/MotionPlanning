#ifndef HW3_HEADER_H
#define HW3_HEADER_H

#include <vector>
#include <iostream>
using namespace std;
using namespace OpenRAVE;
typedef vector<dReal> config;

class RRTNode
{
    public:
        RRTNode *_parent;
        config _configuration;

        RRTNode(const config config1);
        RRTNode(RRTNode* parent,const config config2);

        ~RRTNode(){this->_parent=nullptr;}

        config get_config() const { return _configuration;}

        RRTNode* get_parent() const { return _parent;}
        void print();
};

class NodeTree
{
    private:
        RRTNode* root;
        //vector<RRTNode*> all_nodes;

    public:
        NodeTree(RRTNode*);
        ~NodeTree(){};
        vector<RRTNode*> all_nodes;

        void add_node(RRTNode* prev,RRTNode* node);

        RRTNode* nearest_neigh(RRTNode* nod);

        double dist(RRTNode* n1, RRTNode* n2);
        double euc_dist(config d1, config d2);

        bool connect(config &rand_con, RRTNode* near_con, config v);

        bool check_collision(RRTNode*);

        void print_data(RRTNode*);
        void print_allnode();
        vector<RRTNode*> get_path(RRTNode* node);
        vector<RRTNode*> get_path_reverse(RRTNode* node);
        RRTNode* last_node();
        bool check_joint_limit(RRTNode* N);
        void delete_node();

        //RRTNode* get_node() const {return  ;}
//    size_t get_tree_size() const {return _nodes.size();};
//    bool delete_node(RRTNode* node);

};
#endif
