/* -------------------------------------------------------------------------- */
/*                                  INCLUDES                                  */
/* -------------------------------------------------------------------------- */

/* ----------------------------------- ROS ---------------------------------- */
#include "ros/ros.h"
/* -------------------------------------------------------------------------- */

/* ---------------------------------- Eigen --------------------------------- */
#define EIGEN_RUNTIME_NO_MALLOC
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
/* -------------------------------------------------------------------------- */

/* ----------------------------- Cubic B-Spline ----------------------------- */
#include "alglib_pkg/interpolation.h"
/* -------------------------------------------------------------------------- */

/* -------------------------------- Algorithm ------------------------------- */
#include <algorithm> 
/* -------------------------------------------------------------------------- */

/* --------------------------------- Format --------------------------------- */
#define FMT_HEADER_ONLY
#include <fmt/format.h> // target_link_libraries(<target> fmt)
/* -------------------------------------------------------------------------- */

/* ------------------------------- Scooped Ptr ------------------------------ */
#include <boost/shared_ptr.hpp>
/* -------------------------------------------------------------------------- */

/* -------------------------- Kinematics Functions -------------------------- */
#include "auxiliary_functions_pkg/kinematics_auxiliary_functions.h"
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                    ENUM                                    */
/* -------------------------------------------------------------------------- */
enum class activityStates{NOT_STARTED_YET,STARTED,COMPLETED};
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                               DATA STRUCTURE                               */
/* -------------------------------------------------------------------------- */
struct controllerStruct
{
    /* -------------------------------------------------------------------------- */
    /*                             Control Parameters                             */
    /* -------------------------------------------------------------------------- */
    double b;

    double y_1;
    double y_2;

    double dy_1;
    double dy_2;

    double y_1d;
    double y_2d;

    double dy_1d;
    double dy_2d;

    double error;

    double distance_threshold;

    bool obstacle_avoidance_enabled;
    double obstacle_avoidance_threshold;
    double obstacle_avoidance_gain;
    double obstacle_avoidance_eta;
    std::vector<Eigen::Vector2d> obstacles_position;

    Eigen::Vector2d avoidance_velocities;

    double k_1;
    double k_2;
    double k_3;

    double max_linear_velocity;
    double max_angular_velocity;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                              Spline Trajectory                             */
    /* -------------------------------------------------------------------------- */
    alglib::real_1d_array times;
    alglib::real_1d_array traj_x_values;
    alglib::real_1d_array traj_y_values;

    alglib::ae_int_t bound_type = 1; // *  1, which corresponds to the first derivative boundary condition
    alglib::ae_int_t N = 9;

    alglib::spline1dinterpolant traj_x;
    alglib::spline1dinterpolant traj_y;

    Eigen::Vector2d s;
    Eigen::Vector2d ds;
    Eigen::Vector2d dds;
    /* -------------------------------------------------------------------------- */
};

struct goalStruct
{
    /* --------------------------------- Fields --------------------------------- */
    int type;

    Eigen::Vector3d initial_pose;
    Eigen::Vector3d mid_pose;
    Eigen::Vector3d final_pose;

    double displacement;
    Eigen::Vector3d times;
    /* -------------------------------------------------------------------------- */
};

struct phaseStruct
{
    /* --------------------------------- Fields --------------------------------- */
    std::string name;
    int type; // 0 - Moving phase (Forward Motion with L-path), 1 - Moving phase (Backward Motion with L-path), 2 - Moving phase (Motion with Linear Path), -1 - Waiting / Serving phase
    double startTime;
    double endTime;
    double allocated_time;
    Eigen::Vector3d desired_configuration;
    std::pair<bool,bool> switch_condition;
    goalStruct goal;
    /* -------------------------------------------------------------------------- */
};

struct activityStruct
{
    /* --------------------------------- Fields --------------------------------- */
    double nominal_start_time;
    double start_time;
    activityStates state = activityStates::NOT_STARTED_YET;
    std::vector<phaseStruct> phase;
    /* -------------------------------------------------------------------------- */
};

struct ongoingActivityStruct
{
    /* --------------------------------- Fields --------------------------------- */ 
    double start_time;
    activityStruct activity;
    /* -------------------------------------------------------------------------- */
};
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                    CLASS                                   */
/* -------------------------------------------------------------------------- */
class RobotControllerClass
{
    private:
    /* ------------------------------- Attributes ------------------------------- */
    std::string name;
    int ID;
    
    Eigen::Vector2d home_configuration;

    bool odometry_state;

    Eigen::Vector2d position;
    double theta;

    double linear_velocity;
    double angular_velocity;

    controllerStruct robot_controller;

    bool ongoing_activity_;
    int ongoing_activity_ID;
    int ongoing_phase_ID;

    std::vector<activityStruct> activities;
    ongoingActivityStruct* ongoing_activity;
    /* -------------------------------------------------------------------------- */

    public:

    /* ------------------------------ Constructors ------------------------------ */
    RobotControllerClass(std::string name_, int ID_, Eigen::Vector2d home_configuration_);
    /* -------------------------------------------------------------------------- */

    /* ------------------------------- Destructor ------------------------------- */
    ~RobotControllerClass();
    /* -------------------------------------------------------------------------- */

    /* ----------------------------- Initialization ----------------------------- */
    void init_robot_controller_parameters(double b_, double distance_threshold_, double k_1_, double k_2_, double k_3_, double max_linear_velocity_, double max_angular_velocity_, bool obstacle_avoidance_enabled_, double obstacle_avoidance_gain_, double obstacle_avoidance_threshold_, double obstacle_avoidance_eta_);

    void init_spline_trajectories(Eigen::Vector2d start_position, Eigen::Vector2d corner_position, Eigen::Vector2d end_position, double t_1, double t_2, double t_3);
    /* -------------------------------------------------------------------------- */

    /* ------------------------------ Update/Reset ------------------------------ */
    void update_robot_parameters(const double& t);
    void update_obstacles_position(std::vector<Eigen::Vector2d> obstacles_position_);
    /* -------------------------------------------------------------------------- */

    /* ----------------------------------- Set ---------------------------------- */
    void set_name(std::string name_);
    void set_ID(int ID_);

    void set_home_configuration(Eigen::Vector2d home_configuration_);

    void set_odometry_state(bool state_);

    void set_pose(Eigen::Vector2d position_, double theta_);
    /* -------------------------------------------------------------------------- */

    /* ----------------------------------- Get ---------------------------------- */
    std::string get_name();
    int get_ID();

    int get_ongoing_activity_ID();
    int get_ongoing_phase_ID();

    Eigen::Vector2d get_home_configuration();

    bool get_odometry_state();

    Eigen::Vector2d get_position();
    double get_orientation();

    double get_linear_velocity();
    double get_angular_velocity();

    Eigen::Vector2d get_control_point_position();
    Eigen::Vector2d get_control_point_velocity();

    Eigen::Vector2d get_desired_control_point_position();
    Eigen::Vector2d get_desired_control_point_velocity();

    Eigen::Vector2d get_avoidance_velocity_offsets();
    
    double get_control_error();

    ongoingActivityStruct get_ongoing_activity();
    std::vector<activityStruct> get_activities();
    /* -------------------------------------------------------------------------- */

    /* ------------------------------ Other Methods ----------------------------- */
    void allocate_a_plan(std::vector<activityStruct> activities_);
    goalStruct goal_generation(int type_, Eigen::Vector3d initial_pose_, Eigen::Vector3d final_pose_, double allocated_time_, double starting_time_);
    void evaluate_spline_and_derivatives(Eigen::Vector2d starting_position_, double starting_time_, Eigen::Vector2d final_position_, double final_time_, double time);
    /* -------------------------------------------------------------------------- */
};
/* -------------------------------------------------------------------------- */