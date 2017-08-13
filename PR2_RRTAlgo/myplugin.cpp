#include <openrave/plugin.h>
#include <openrave/openrave.h>
#include <openrave/planningutils.h>
#include <openrave/utils.h>
#include <boost/bind.hpp>
#include "hw3_header.h"
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <time.h>
#include <fstream>
using namespace OpenRAVE;
using namespace std;

vector<RobotBasePtr> _r;
EnvironmentBasePtr en;
RobotBasePtr robot;
vector <RRTNode*> path;
vector<config> RRT_path;

double m = 0.25;
const int dim = 7;
const double lower_limit[7] = {-5.64601796e-01,-3.53600216e-01,-2.12130808e+00,-6.50000756e-01,-3.14,-2.00000770e+00,-3.14};
const double upper_limit[7] = {2.13539289e+00,1.29629967e+00,-1.50000054e-01,3.74999698e+00,3.14,-1.00000036e-01,3.14};
const double goalbias=0.16;
const double weigh[7]={3.17104,2.75674,2.2325, 1.78948,0,0.809013,0};
config s = {-0.15,0.075,-1.008,-0.11,0,-0.11,0};
config g ;

RRTNode::RRTNode(const config config1)
{
    this->_configuration=config1;
    this->_parent=NULL;
}

RRTNode::RRTNode(RRTNode* parent,const config config2)
{
    this->_configuration=config2;
    this->_parent=parent;
}

void RRTNode::print()
{
    for(int j = 0; j < dim; j++)
    {
        cout << this->_configuration[j] << ", ";
    }
    cout<<endl;
}



NodeTree::NodeTree(RRTNode* r){

    this->root=r;
    all_nodes.push_back(r);

}

void NodeTree::add_node(RRTNode* prev, RRTNode* node)
{
    all_nodes.push_back(node);
    node->_parent=prev;
}

RRTNode* NodeTree::nearest_neigh(RRTNode* nod)
{
    RRTNode* near=nullptr;
    size_t j=1;
    double d;

    d = this->dist(all_nodes[0],nod);
    near = this->root;

    near = all_nodes[0];
    while(j != all_nodes.size())
    {
        if (dist(all_nodes[j],nod) < d)
        {
            d = dist(all_nodes[j], nod);
            near = all_nodes[j];
        }
        j++;
    }
    return near;
}

double NodeTree::dist(RRTNode* n1, RRTNode* n2)
{
    config d1,d2;
    d1=n1->get_config();
    d2=n2->get_config();
    //return weigh_dist(d1,d2);
    double dist=0.0;
    for(size_t j=0;j<d1.size();j++)
    {
        dist+=pow((d1[j]-d2[j])*weigh[j],2);
    }
    return sqrt(dist);
}

double NodeTree::euc_dist(config d1, config d2)
{
    double dist = 0.0;
    for(size_t j = 0; j < d1.size(); j++)
    {
        dist += pow((d1[j] - d2[j]), 2);
    }
    return sqrt(dist);
}

bool NodeTree::connect(config &rand_con, RRTNode* near_con, config v)
{

    config r = rand_con;
    config n = near_con->get_config();
    config nod_new;
    config z = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    RRTNode* next = nullptr;
    while(euc_dist(n, r) > (m))
    {

        nod_new.clear();
        for(size_t j = 0; j < r.size(); j++)
        {
            nod_new.push_back(n[j] + m*v[j]);
        }

        next = new RRTNode(nod_new);

        if ((!check_collision(next))&& (check_joint_limit(next)))
        {
            this->add_node(near_con, next);
            near_con = next;
            n = nod_new;
        }
        else
        {
            return false;
        }

    }
        next = new RRTNode(rand_con);
        if ((!check_collision(next)) && (check_joint_limit(next)))
        {
            this->add_node(near_con, next);
        }
        return true;
}

bool NodeTree::check_joint_limit(RRTNode* N)
{
    bool joint;
    config s=N->_configuration;
    for(int j=0;j<dim;j++)
    {
        if (upper_limit[j]>=s[j] && s[j]>=lower_limit[j])
        {
            joint=true;
        }
        else
        {
            joint=false;
        }
    }
    return joint;


}

bool NodeTree::check_collision(RRTNode* N)
{
    bool check;
    robot->SetActiveDOFValues(N->_configuration);
    if ((en->CheckCollision(robot)) ||( robot->CheckSelfCollision()))
    {
        check=true;
    }
    else
    {
        check=false;
    }
    return check;
}


void NodeTree::print_data(RRTNode* g)
{
    RRTNode* temp=g;

    while(temp!=nullptr){
        temp->print();
        cout << endl;
        temp=temp->get_parent();
    }
}

void NodeTree::print_allnode()
{
    cout<<"ALL NODES"<<endl;
    for (size_t j=0;j<all_nodes.size();j++)
    {
        all_nodes[j]->print();
        cout<<endl;
    }
}

vector<RRTNode*> NodeTree::get_path(RRTNode* node)
{
    RRTNode* temp=node;
    vector <RRTNode*> v;
    while(temp!=nullptr){
        v.push_back(temp);
        temp=temp->get_parent();
    }

    return v;
}

vector<RRTNode*> NodeTree::get_path_reverse(RRTNode* node)
{
    RRTNode* temp=node;
    vector <RRTNode*> v;
    while(temp!=nullptr)
    {
        v.push_back(temp);
        temp=temp->get_parent();
    }
    reverse(v.begin(),v.end());

    return v;
}

RRTNode* NodeTree::last_node()
{
    return all_nodes.back();

}

void NodeTree::delete_node()
{
    for(size_t i=0;i<all_nodes.size();i++)
    {
        //all_nodes.remove(i);
        free(all_nodes[i]);
    }
}

double rand_new()
{
    double r;
    r = float(rand())/float(RAND_MAX);
    return r;
}

config random_config()
{
    config c1;
    for(int j=0;j<dim;j++)
    {
        c1.push_back((upper_limit[j]-lower_limit[j])*rand_new()+lower_limit[j]);
    }
    return c1;

}

config unit_vector(RRTNode* rand_con, RRTNode* near_con)
{
    config dist_vect = rand_con->get_config();
    config near_vect = near_con->get_config();

    config u_vect;
    config diff_vect;
    NodeTree t(nullptr);
    double d = t.euc_dist(dist_vect, near_vect);

    for(int j = 0; j < dim; j++)
    {
        diff_vect.push_back(dist_vect[j] - near_vect[j]);
        u_vect.push_back(diff_vect[j]/d);
    }
    return u_vect;
}

//void pri_file()
//{
//    ofstream file;
//    file.open("hw3.csv",ios_base::app);
//    file<<fixed<<setprecision(2)<<count
//}




class MyNewModule : public ModuleBase
{
public:
    MyNewModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        en=penv;
        en->GetRobots(_r);
        robot=_r.at(0);
        RegisterCommand("RRT_connect",boost::bind(&MyNewModule::RRT_connect,this,_1,_2),
                        "This is an example command");
    }
    virtual ~MyNewModule() {}
    vector<GraphHandlePtr> handler;
    vector<dReal> get_input(ostream& sout, istream& sinput)
    {
        char input[70];
        vector<dReal> g;
        try
        {
              vector<string> a;
              sinput.getline(input, 70);
              utils::TokenizeString(input,"[ ]", a);
              for(string s : a)
                g.push_back(atof(s.c_str()));
        }
        catch(exception &e)
        {
          cout << e.what() << endl;
        }
        return g;
    }

    void execute_path(vector<RRTNode*> path)
    {


        EnvironmentMutex& mute = en->GetMutex();
        mute.lock();
        TrajectoryBasePtr t = RaveCreateTrajectory(en);
        t->Init(robot->GetActiveConfigurationSpecification());

        for(RRTNode* p : path)
        {
             t->Insert(0, p->_configuration);
        }

        planningutils::RetimeActiveDOFTrajectory(t,robot);

        robot->GetController()->SetPath(t);

        mute.unlock();
    }


    void draw_end_effector(vector<RRTNode*> pat,int color)
    {
        vector<float> red = {1,0,0,1};
        vector<float> blue= {0,0,1,1};
        for (size_t j=0;j<pat.size();j++)
        {

            RRT_path.push_back(pat[j]->_configuration);
            robot->SetActiveDOFValues(pat[j]->_configuration);
            robot->SetActiveManipulator("leftarm");
            RobotBase::ManipulatorPtr man;
            man = robot->GetActiveManipulator();
            RaveVector<dReal> point = man->GetEndEffectorTransform().trans;
            vector<float> end_effector;
            end_effector.push_back(point.x);
            end_effector.push_back(point.y);
            end_effector.push_back(point.z);
            if(color == 1)
            {
                handler.push_back(en->plot3(&end_effector[0],1,1,5,&red[0],0,true));
            }
            else
            {
                handler.push_back(en->plot3(&end_effector[0],1,1,5,&blue[0],0,true));
            }

        }
        //cout<<end_effect[0]<<endl;

    }

    bool RRT_connect(std::ostream& sout, std::istream& sinput)
    {

            clock_t stat,last;
            stat=clock();
            double cpu_time_used=0.0;
            g=get_input(sout,sinput);
            srand(time(NULL));
            config u, r;
            config z = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
           // double count;
            RRTNode* start = new RRTNode(s);
            RRTNode* k = nullptr;
            NodeTree RRT(start);
          //  vector <RRTNode*> path;
            RRTNode* random;
            bool goal_flag=false;

            while(true)
                {

                    //count++;
                    if ((rand_new()<goalbias))
                    {
                        goal_flag=1;
                        r=g;
                    }
                    else
                    {
                        goal_flag=0;
                        r = random_config();
                    }
                    random = new RRTNode(r);
                    k = RRT.nearest_neigh(random);
                    u = unit_vector(random, k);
                    if( RRT.connect(r, k, u) && goal_flag)
                        break;


                }

                //RRT.print_allnode();
    //            cout << "PATH" << endl;
                path = RRT.get_path(RRT.last_node());
                //file<<"path size"<<path.size()<<endl;
    //            for (size_t j = 0; j < path.size()-1; j++)
    //            {
    //                config p=path[j]->_configuration;
    //                draw_end_effector(*p);
    //
    //                //cout << endl;
    //            }



                draw_end_effector(path,1);
                execute_path(path);
                //ofstream file;
                //file.open("hw3.csv",ios_base::app);
                //file<<count<<","<<"\n";
               // ofstream file1;
               // file1.open("hw3.csv",ios_base::app);
                //file1<<path.size()<<","<<"\n";
                //RRT.delete_node();
                //shortcut_smooth();
                last=clock();
                cpu_time_used+=(double(last-stat))/CLOCKS_PER_SEC;
               // ofstream file2;
                //file2.open("hw3.csv",ios_base::app);
                //file2<<cpu_time_used<<","<<"\n";


            return true;


    }


    void shortcut_smooth()
{
    //cout<<RRT_path.size()<<endl;
    vector <RRTNode*> smooth_tree_path;
    vector <RRTNode*> pat;
    config unit_s;
    RRTNode* t=nullptr;

    RRTNode* fist = new RRTNode(RRT_path[0]);
    NodeTree old_tree(fist);
    t=fist;

    for(size_t j=1;j<RRT_path.size();j++)
    {

        RRTNode* nex = new RRTNode(RRT_path[j]);
        old_tree.add_node(t,nex);
        //nex->print();
        t=nex;

        //cout<<"tree constructed"<<endl;

    }
    pat=old_tree.get_path(old_tree.last_node());
    //reverse(pat.begin(),pat.end());
    cout<<"pat size"<<pat.size()<<endl;
    for(int i=0;i<200;i++)
    {
        int a=rand_new()*pat.size();
        int b=rand_new()*pat.size();
       // cout<<"a"<<pat[a]<<endl;
        //cout<<"b"<<pat[b]<<endl;
        NodeTree smooth_tree(pat[a]);
        unit_s = unit_vector(pat[a],pat[b]);
        if(smooth_tree.connect(pat[a]->_configuration,pat[b],unit_s))
        {
            cout<<"smooth size1"<<smooth_tree_path.size()<<endl;
            smooth_tree_path = smooth_tree.get_path(smooth_tree.last_node());
            for (size_t i;i<smooth_tree_path.size();i++)
            {
                if (smooth_tree_path[i]==pat[b])
                {
                    RRTNode* tem=smooth_tree_path[i]->_parent;
                    pat[b]->_parent=nullptr;
                    pat[b]=tem;
                }
                if(smooth_tree_path[i]==pat[a])
                {
                    RRTNode* tem1=pat[a]->_parent;
                    smooth_tree_path[i]->_parent=tem1;
                    pat[a]->_parent=nullptr;
                }
            }
        }

            cout<<"NO. OF ITERATION"<<i<<endl;
            cout<<"LengtH of paTH"<<smooth_tree_path.size()<<endl;


    }
   // cout<<"smooth size out"<<smooth_tree_path.size()<<endl;
    reverse(smooth_tree_path.begin(),smooth_tree_path.end());
    execute_path(smooth_tree_path);
    draw_end_effector(smooth_tree_path,2);





}
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "mynewmodule" ) {
        return InterfaceBasePtr(new MyNewModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("MyNewModule");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

