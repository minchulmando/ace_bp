#pragma once
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <stdint.h>
#include <vector>
#include <string>
#include <brain_fm/LifeSignal.h>

namespace BRAIN{
    static const int ID_MODULE_POSE_ESTIMATION = 1;
    static const int ID_MODULE_MAP_FINDER = 2;
    static const int ID_MODULE_LOCAL_ROUTE = 3;
    static const int ID_MODULE_OBJECT_FUSION = 4;
    static const int ID_MODULE_BEHAVIOR_PLANNING = 5;
    static const int ID_MODULE_LOCAL_PLANNING = 6;
    static const int ID_MODULE_LON_CONTROL = 7;
    static const int ID_MODULE_LAT_CONTROL = 8;
    static const int ID_MODULE_LANE_FUSION = 9;
    static const int ID_MODULE_MAP_MATCHING = 10;
    static const int ID_MODULE_TLR = 11;
    static const int ID_MODULE_OBJECT_TRAJECTORY = 12;
    static const int ID_MODULE_LANE_TRAJECTORY = 13;
    static const int ID_MODULE_ROUTE_MAPPING = 14;
}

class FaultDetector{
    public:
	FaultDetector();
	~FaultDetector();

    private:
	int module_id_;
	std::string module_name_;

	bool fault_task_ = false;
	bool fault_function_ = false;
	bool fault_device_ = false;

    private:
	std::vector<std::string> fault_device_list_;

    public:
	void Init( int id, std::string name );
	void MainLoop();

    private:
	const double CHECK_PERIOD = 0.5;
	ros::Publisher pub_life_;
	void SendLifeSignal();	

    public:
	inline void SetFaultFunc() { fault_function_ = true; }
	inline void SetFaultTask() { fault_task_ = true; }
	inline void SetFaultDevice() { fault_device_ = true; }

	inline void SetNormalFunc() { fault_function_ = false; }
	inline void SetNormalTask() { fault_task_ = false; }
	inline void SetNormalDevice(){ fault_device_ = false; }

    private:
	const uint8_t ERROR_NONE = 0x00;
	const uint8_t ERROR_FUNC = 0x02;
	const uint8_t ERROR_TASK = 0x04;
	const uint8_t ERROR_DEVICE = 0x08;

    private:
	brain_fm::LifeSignal life_signal_;
};

FaultDetector::FaultDetector():
module_id_( 0 ),
module_name_(""){
}

FaultDetector::~FaultDetector(){
}

void FaultDetector::Init( int id, std::string name ){
   module_id_ = id;
   module_name_= name;

   ros::NodeHandle nh;
   pub_life_ = nh.advertise<brain_fm::LifeSignal>("fm/" + name, 10);
}

void FaultDetector::MainLoop(){
    while( ros::ok() ){
	SendLifeSignal();
	ros::Duration(CHECK_PERIOD).sleep();
    }
}

void FaultDetector::SendLifeSignal(){
    brain_fm::LifeSignal life;
    life.header.stamp = ros::Time::now();
    life.id = module_id_;
    life.module = module_name_;

    int fault_code = ERROR_NONE;
    life.is_fault = false;
    if( fault_task_==true) {
	life.is_fault = true;
	fault_code = fault_code + ERROR_TASK;
    }	 
    if(fault_function_==true){
	life.is_fault = true;
	fault_code = fault_code + ERROR_FUNC;
    }
    if(fault_device_==true){
	life.is_fault = true;
	fault_code = fault_code + ERROR_DEVICE;
    }
    life.error_code = fault_code;

    pub_life_.publish( life );
}
