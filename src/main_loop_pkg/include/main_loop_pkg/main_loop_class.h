/* -------------------------------------------------------------------------- */
/*                                INCLUDE FILES                               */
/* -------------------------------------------------------------------------- */

/* ----------------------------------- ROS ---------------------------------- */
#include "ros/ros.h"
/* -------------------------------------------------------------------------- */

/* -------------------------------- Messages -------------------------------- */
#include "nav_msgs/Odometry.h"
#include "diff_drive_robot/MILPResult.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
/* -------------------------------------------------------------------------- */

/* ---------------------------------- Eigen --------------------------------- */
#define EIGEN_RUNTIME_NO_MALLOC
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
/* -------------------------------------------------------------------------- */

/* --------------------------------- Format --------------------------------- */
#define FMT_HEADER_ONLY
#include <fmt/format.h> // target_link_libraries(<target> fmt)
/* -------------------------------------------------------------------------- */

/* ---------------------------------- Robot --------------------------------- */
#include "robot_controller_pkg/robot_controller_class.h"
/* -------------------------------------------------------------------------- */

/* ---------------------------------- Pair ---------------------------------- */
#include <utility>
/* -------------------------------------------------------------------------- */

/* ----------------------------------- Xml ---------------------------------- */
#include <XmlRpcValue.h>
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                   DEFINES                                  */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/*                                    CLASS                                   */
/* -------------------------------------------------------------------------- */
#ifndef MainLoopClass_H
#define MainLoopClass_H
class MainLoopClass
{
    private:
    /* ------------------------------ Node Handler ------------------------------ */
    ros::NodeHandle node_handler;
    /* -------------------------------------------------------------------------- */

    /* ---------------------------------- Timer --------------------------------- */
    ros::Timer periodic_timer;
    /* -------------------------------------------------------------------------- */

    /* ------------------------------- Subscribers ------------------------------ */
    std::vector<ros::Subscriber> odometry_SUBs;
    std::vector<ros::Subscriber> MILP_results_SUBs;
    ros::Subscriber timing_SUB;
    /* -------------------------------------------------------------------------- */

    /* ------------------------------- Publishers ------------------------------- */
    std::vector<ros::Publisher> robots_velocities_PUBs;

    std::vector<ros::Publisher> control_points_pose_PUBs;
    std::vector<ros::Publisher> desired_control_points_pose_PUBs;

    std::vector<ros::Publisher> control_points_velocity_PUBs;
    std::vector<ros::Publisher> desired_control_points_velocity_PUBs;

    std::vector<ros::Publisher> avoidance_velocity_offsets_PUBs;

    std::vector<ros::Publisher> robots_ongoing_activity_PUBs;
    /* -------------------------------------------------------------------------- */

    /* --------------------------- Control Attributes --------------------------- */
    bool simulation;

    std::pair<bool,bool>external_timer; //<Use an external timer,Has the timer been initialized?>
    double control_frequency;
    double Ts;
    double t;
    
    int number_of_robots;

    std::vector<Eigen::Vector2d> home_configurations_vector;

    double b;

    double k_1;
    double k_2;
    double k_3;

    double max_linear_velocity;
    double max_angular_velocity;

    int number_of_humans;
    std::vector<Eigen::Vector2d> humans_positions_vector;

    bool obstacle_avoidance_enabled;
    double obstacle_avoidance_threshold;
    double obstacle_avoidance_gain;
    double obstacle_avoidance_eta;
    double distance_threshold;

    std::vector<RobotControllerClass> robots_vector;

    std::vector<int> plan_IDs;
    /* -------------------------------------------------------------------------- */

    /* ------------------------- Initialization Methods ------------------------- */
    void init_robots_parameters();
    void init_humans_parameters();
    void init_controllers_parameters();
    void init_publishers_and_subscribers();
    /* -------------------------------------------------------------------------- */

    /* --------------------------- Periodic Callbacks --------------------------- */
    void init_timer_callback(const ros::TimerEvent &event);
    void periodic_timer_callback(const ros::TimerEvent &event);
    /* -------------------------------------------------------------------------- */

    /* --------------------------- Callback Functions --------------------------- */
    void time_update(const std_msgs::Float64::ConstPtr &msg);
    void odometry_subscriber_simulation_CB(const nav_msgs::Odometry::ConstPtr &msg, int robot_ID);
    void odometry_subscriber_real_CB(const geometry_msgs::PoseStamped::ConstPtr &msg, int robot_ID);
    void MILP_results_subscriber_CB(const diff_drive_robot::MILPResult::ConstPtr &msg, int robot_ID);
    /* -------------------------------------------------------------------------- */

    /* -------------------------------- Messages -------------------------------- */
    void publish_robots_velocities();
    void publish_avoidance_velocity_offsets();

    void publish_control_points_poses();
    void publish_control_points_velocities();

    void publish_robots_ongoing_activity();
    /* -------------------------------------------------------------------------- */

    public:

    /* ----------------------- Constructors and Destructor ---------------------- */
    MainLoopClass(const ros::NodeHandle &node_handler_);
    ~MainLoopClass();
    /* -------------------------------------------------------------------------- */
};
#endif
/* -------------------------------------------------------------------------- */