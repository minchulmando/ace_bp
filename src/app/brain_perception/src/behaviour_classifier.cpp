#include <ros/package.h>
#include "behaviour_classifier.h"

TaskNode::TaskNode(int id, std::string task_node, double period)
:AtomTask( id, task_node, period )
{
}

TaskNode::~TaskNode(){
}

void TaskNode::Init(){
    // config ini file initialization
    std::string current_path = ros::package::getPath("brain_perception");
	std::string conf_path("/config/config_behaviour.ini");
    std::string ini_path = current_path + conf_path;
	
    if(config_parser_.Init(ini_path.c_str())){
        ROS_INFO("The configuration file was loaded (%s)",ini_path.c_str());
    }else{
        ROS_ERROR("The configuration file cannot be loaded at given path (%s)",ini_path.c_str());
        ros::shutdown();
    }
}

void TaskNode::Run()
{
    // estimate the time difference
    static ros::WallTime t_archive = ros::WallTime::now();
    double dt = (ros::WallTime::now() - t_archive).toNSec() * 1e-9;
    t_archive = ros::WallTime::now();

	// wirte your code here









    double dt_exc = (ros::WallTime::now() - t_archive).toNSec() * 1e-9;
    ROS_INFO("[Excution] algorithm run time(exc/per): %.5f / %.5f [sec]", dt_exc, dt);
}

void TaskNode::Publish(){

}

void TaskNode::Terminate(){
}

int main(int argc, char **argv){
    
    ros::Time::init();
    std::string node_name = "behaviour_classifier";
    ros::init(argc, argv, node_name);

    TaskNode main_route_mapping( BRAIN::ID_MODULE_ROUTE_MAPPING, node_name.c_str(), APP_TASK_PERIOD );
    main_route_mapping.Exec( APP_TASK_THREADS );

    return 0;
}

void TaskNode::ProcessINI(void){
    if(config_parser_.IsFileUpdated() == true){
        

        ROS_INFO("The configuration is newly updated now.");
    }
}