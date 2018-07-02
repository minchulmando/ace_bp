#ifndef __behaviour_classifier__
#define __behaviour_classifier__

// ros basic core
#include <ros/ros.h>
#include <ros/console.h>
#include "../../brain_common/atom_task/atom_task.h"
#include "ini_parser/ini_parser.h"

const double APP_TASK_PERIOD = 0.1;
const int APP_TASK_THREADS = 1;

class TaskNode : public AtomTask
{
    public:
        TaskNode(int id, std::string task_node, double period);
        ~TaskNode();

    public: // system basic functions
        void Init(void);
        void Run(void);
        void Publish(void);
        void Terminate(void);
        void ProcessINI(void);

    private:
        // system handlers
        ros::NodeHandle nh_;
        CINI_Parser config_parser_;
};

#endif
