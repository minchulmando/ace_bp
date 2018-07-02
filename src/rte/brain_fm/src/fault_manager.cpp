// ######## ROS ###########
#include <ros/ros.h>

// ####### MESSAGES ######
#include <brain_fm/LifeSignal.h>
#include <brain_fm/SystemFault.h>

#include <boost/thread/thread.hpp>

#include <map>
#include <mutex>

#define FAULT_CHECK_PERIOD 1
#define MAX_NUM_MODULE 20
#define WATCHDOG_THRESHOULD 0.5

class fault_manager{
    public:
	fault_manager();
	~fault_manager();


	// ###################
	// Task
    public:
	void MainLoop();

    public:
	void Init();
	void Run();
	void Publish();

	// ###################
	// fault maanagement
    private:
	int num_module_ = 0;
	std::map<int,int> fault_check_list_;

	void RegisterFM( int module_id, std::string msg_name );

    private:
	ros::Publisher pub_fault_;
	ros::Subscriber sub_fm_[MAX_NUM_MODULE];

    public:
	void CallbackFaultMessage( const brain_fm::LifeSignal::ConstPtr& msg );

    private:
	brain_fm::LifeSignal fm_msgs_O_[MAX_NUM_MODULE];
	brain_fm::LifeSignal fm_msgs_[MAX_NUM_MODULE];

	std::string module_name_list_[MAX_NUM_MODULE];

	std::mutex mutex_fm_msgs_[MAX_NUM_MODULE];

	brain_fm::SystemFault fault_msg_;

    private:
	void CheckFault(brain_fm::SystemFault& updated_fault);
};

fault_manager::fault_manager(){
}

fault_manager::~fault_manager(){
}

void fault_manager::RegisterFM( int module_id, std::string msg_name ){
    ros::NodeHandle nh;
    fault_check_list_.insert( std::pair<int,int>(module_id, num_module_) );
    sub_fm_[num_module_] = nh.subscribe( msg_name, 0, &fault_manager::CallbackFaultMessage, this );
    module_name_list_[num_module_] = msg_name;
    num_module_++;
}

void fault_manager::MainLoop(){
    ros::Rate loop_rate( FAULT_CHECK_PERIOD );
    while( ros::ok() ){
	Run();
	Publish();
	ros::spinOnce();
	loop_rate.sleep();
    }
}

void fault_manager::Init(){
    ros::NodeHandle nh;
    pub_fault_ = nh.advertise< brain_fm::SystemFault > ("fm/fault", 10);

    RegisterFM( 1, "fm/ego_pos_estimator" );
    RegisterFM( 2, "fm/map_finder" );
    RegisterFM( 3, "fm/local_route" );
    RegisterFM( 4, "fm/object_fusion" );
    RegisterFM( 5, "fm/behavior_planning" );
    RegisterFM( 6, "fm/local_planner" );
    RegisterFM( 7, "fm/lon_control" );
    RegisterFM( 8, "fm/lat_control" );
    RegisterFM( 10, "fm/map_matching" );
    RegisterFM( 11, "fm/tlr" );
}

void fault_manager::Run(){
    CheckFault(fault_msg_);
}

void fault_manager::Publish(){
    pub_fault_.publish( fault_msg_ );
}

void fault_manager::CallbackFaultMessage( const brain_fm::LifeSignal::ConstPtr& msg){
    std::map<int,int>::iterator it;
    it = fault_check_list_.find( msg->id );
    if( it != fault_check_list_.end() ){
	// success to find
	int id = (int)it->second;
	mutex_fm_msgs_[id].lock();
	fm_msgs_O_[id] = fm_msgs_[id];
	fm_msgs_[id] = *msg;
	mutex_fm_msgs_[id].unlock();

	std::cout << "id: " << id << std::endl;
    }

    std::cout << "message is received" << std::endl;
}

void fault_manager::CheckFault(brain_fm::SystemFault& updated_fault){
    brain_fm::LifeSignal fm_msgs[MAX_NUM_MODULE];
    brain_fm::LifeSignal fm_msgsO[MAX_NUM_MODULE];

    for( int i=0;i<num_module_; i++){
	mutex_fm_msgs_[i].lock();
	fm_msgs[i] = fm_msgs_[i];
	fm_msgsO[i] = fm_msgs_O_[i];
	mutex_fm_msgs_[i].unlock();
    }

    brain_fm::SystemFault fault_msg;
    for( int i=0; i<num_module_; i++){

	std::cout << "array id: " << i << std::endl;
	// check watchdog
	int watchdog_flag = 0x01;
	ros::Duration dif_time = fm_msgs[i].header.stamp - fm_msgsO[i].header.stamp;
	ros::Duration last_time = ros::Time::now() - fm_msgs[i].header.stamp;

	std::cout << last_time.toSec() << std::endl;

	std::cout << "check watchdog" << std::endl;
	bool is_watchdog = false;
	if( last_time.toSec() > WATCHDOG_THRESHOULD ){
	    is_watchdog = true;
	}

	std::cout << "check fault" << std::endl;
	if( (fm_msgs[i].is_fault == true ) ||
		(is_watchdog == true)){
	    fault_msg.is_fault = true;

	    brain_fm::FaultElement ele;
	    ele.module_name = module_name_list_[i];
	    ele.error_code = fm_msgs[i].error_code;
	    if( is_watchdog == true ) ele.error_code = ele.error_code + watchdog_flag;
	    fault_msg.error_list.push_back( ele );
	}
	std::cout << "===== end of iterations =========" << std::endl;
    }
    std::cout << "copy" << std::endl;
    updated_fault = fault_msg;
    std::cout << "complete to copy" << std::endl;
}

int main( int argc, char **argv){
    std::string node_name = "fault_manager";
    ros::init( argc, argv, node_name );
    ros::NodeHandle nh;

    fault_manager fm;
    fm.Init();
    boost::thread main_thread( boost::bind( &fault_manager::MainLoop, (fault_manager*)&fm ) );

    ros::AsyncSpinner spinner( 3 );
    spinner.start();
    ros::waitForShutdown();

    main_thread.join();

    return 0;
}
