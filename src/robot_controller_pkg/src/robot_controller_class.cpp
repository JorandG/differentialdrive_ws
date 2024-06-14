/* -------------------------------------------------------------------------- */
/*                                INCLUDE FILES                               */
/* -------------------------------------------------------------------------- */

/* ------------------------------- Robot Class ------------------------------ */
#include "robot_controller_pkg/robot_controller_class.h"
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                         Constructors and Destructor                        */
/* -------------------------------------------------------------------------- */
RobotControllerClass::RobotControllerClass(std::string name_, int ID_, Eigen::Vector2d home_configuration_)
{
    /* -------------------------------------------------------------------------- */
    /*                          Attributes Initialization                         */
    /* -------------------------------------------------------------------------- */
    this->name = name_;
    this->ID = ID_;

    this->home_configuration = home_configuration_;

    this->ongoing_activity_ = false;
    this->ongoing_activity_ID = -1;
    this->ongoing_phase_ID = -1;

    this->odometry_state = false;

    this->linear_velocity = 0.0;
    this->angular_velocity = 0.0;

    this->ongoing_activity = nullptr;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                 Processing                                 */
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
}

RobotControllerClass::~RobotControllerClass(){}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                           Initialization Methods                           */
/* -------------------------------------------------------------------------- */
void RobotControllerClass::init_robot_controller_parameters(double b_, double distance_threshold_, double k_1_, double k_2_, double k_3_, double max_linear_velocity_, double max_angular_velocity_, bool obstacle_avoidance_enabled_, double obstacle_avoidance_gain_, double obstacle_avoidance_threshold_, double obstacle_avoidance_eta_)
{
    /* -------------------------------------------------------------------------- */
    /*                             Auxiliary Variables                            */
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                          Variables Initialization                          */
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                 Processing                                 */
    /* -------------------------------------------------------------------------- */
    this->robot_controller.b = b_;

    this->robot_controller.y_1 = this->position(0) + (this->robot_controller.b * cos(this->theta));
    this->robot_controller.y_2 = this->position(1) + (this->robot_controller.b * sin(this->theta));

    this->robot_controller.dy_1 = 0.0;
    this->robot_controller.dy_2 = 0.0;

    this->robot_controller.y_1d = this->robot_controller.y_1;
    this->robot_controller.y_2d = this->robot_controller.y_2;

    this->robot_controller.dy_1d = this->robot_controller.dy_1;
    this->robot_controller.dy_2d = this->robot_controller.dy_2;

    this->robot_controller.error = 0.0;

    this->robot_controller.distance_threshold = distance_threshold_;

    this->robot_controller.obstacle_avoidance_enabled = obstacle_avoidance_enabled_;
    this->robot_controller.obstacle_avoidance_gain = obstacle_avoidance_gain_;
    this->robot_controller.obstacle_avoidance_threshold = obstacle_avoidance_threshold_;
    this->robot_controller.obstacle_avoidance_eta = obstacle_avoidance_eta_;
    
    this->robot_controller.avoidance_velocities.setZero();

    this->robot_controller.k_1 = k_1_;
    this->robot_controller.k_2 = k_2_;
    this->robot_controller.k_3 = k_3_;

    this->robot_controller.max_linear_velocity = max_linear_velocity_;
    this->robot_controller.max_angular_velocity = max_angular_velocity_;
    /* -------------------------------------------------------------------------- */
}

void RobotControllerClass::init_spline_trajectories(Eigen::Vector2d start_position, Eigen::Vector2d corner_position, Eigen::Vector2d end_position, double t_1, double t_2, double t_3)
{
    /* -------------------------------------------------------------------------- */
    /*                             Auxiliary Variables                            */
    /* -------------------------------------------------------------------------- */
    double sharpness;
    Eigen::Vector2d r_12;
    Eigen::Vector2d r_23;

    Eigen::Vector2d point_A;
    double t_A;

    Eigen::Vector2d point_B;
    double t_B;

    Eigen::Vector2d point_C;
    double t_C;

    Eigen::Vector2d point_D;
    double t_D;

    Eigen::Vector2d point_E;
    double t_E;

    Eigen::Vector2d point_F;
    double t_F;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                          Variables Initialization                          */
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                 Processing                                 */
    /* -------------------------------------------------------------------------- */

    /* -------------------------------- Sharpness ------------------------------- */
    sharpness = 0.30*std::min((corner_position - start_position).norm(),(end_position - corner_position).norm());
    /* -------------------------------------------------------------------------- */

    /* ------------------------------ Unit Vectors ------------------------------ */
    r_12 = (corner_position - start_position)/(corner_position - start_position).norm();
    r_23 = (end_position - corner_position)/(end_position - corner_position).norm();
    /* -------------------------------------------------------------------------- */

    /* --------------------------------- Points --------------------------------- */
    point_A = (start_position + corner_position)/2.0;
    point_B = (corner_position + end_position)/2.0;
    point_C = (start_position + point_A)/2.0;
    point_D = (end_position + point_B)/2.0;
    point_E = corner_position - sharpness*r_12;
    point_F = corner_position + sharpness*r_23;

    this->robot_controller.traj_x_values = fmt::format("[{},{},{},{},{},{},{},{},{}]",start_position(0),point_C(0),point_A(0),point_E(0),corner_position(0),point_F(0),point_B(0),point_D(0),end_position(0)).c_str();

    this->robot_controller.traj_y_values = fmt::format("[{},{},{},{},{},{},{},{},{}]",start_position(1),point_C(1),point_A(1),point_E(1),corner_position(1),point_F(1),point_B(1),point_D(1),end_position(1)).c_str();
    /* -------------------------------------------------------------------------- */

    /* ---------------------------------- Times --------------------------------- */
    t_A = (t_1 + t_2)/2.0;
    t_B = (t_2 + t_3)/2.0;
    t_C = (t_1 + t_A)/2.0;
    t_D = (t_3 + t_B)/2.0;
    t_E = t_2 - ((t_2 - t_1)/(corner_position - start_position).norm())*sharpness;
    t_F = t_2 + ((t_3 - t_2)/(end_position - corner_position).norm())*sharpness;

    this->robot_controller.times = fmt::format("[{},{},{},{},{},{},{},{},{}]",t_1,t_C,t_A,t_E,t_2,t_F,t_B,t_D,t_3).c_str();
    /* -------------------------------------------------------------------------- */

    /* -------------------------- Trajectory Generation ------------------------- */
    alglib::spline1dbuildcubic(this->robot_controller.times,this->robot_controller.traj_x_values,this->robot_controller.N,this->robot_controller.bound_type,0.0 /*First Derivative (Velocity) - Initial Value*/,this->robot_controller.bound_type,0.0 /*First Derivative (Velocity) - Final Value*/,this->robot_controller.traj_x);

    alglib::spline1dbuildcubic(this->robot_controller.times,this->robot_controller.traj_y_values,this->robot_controller.N,this->robot_controller.bound_type,0.0 /*First Derivative (Velocity) - Initial Value*/,this->robot_controller.bound_type,0.0 /*First Derivative (Velocity) - Final Value*/,this->robot_controller.traj_y);
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                            Update/Reset Methods                            */
/* -------------------------------------------------------------------------- */
void RobotControllerClass::update_robot_parameters(const double& t)
{
    /* -------------------------------------------------------------------------- */
    /*                             Auxiliary Variables                            */
    /* -------------------------------------------------------------------------- */
    Eigen::Vector2d velocities;
    Eigen::Matrix2d T;
    Eigen::Vector2d u;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                           Activity/Phase - State                           */
    /* -------------------------------------------------------------------------- */
    if(this->ongoing_activity_)
    {
        /* ------------------------------ Assigned Goal ----------------------------- */
        goalStruct& assigned_goal = this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID].goal;
        /* -------------------------------------------------------------------------- */

        /* ------------------------------ Control Error ----------------------------- */
        this->robot_controller.error = (assigned_goal.final_pose.head(2) - Eigen::Vector2d(this->robot_controller.y_1,this->robot_controller.y_2)).norm();
        /* -------------------------------------------------------------------------- */

        /* --------------------------------- Status --------------------------------- */
        if((this->robot_controller.error <= this->robot_controller.distance_threshold && t >= assigned_goal.times(2)) || t >= assigned_goal.times(2))
        {
            /* ----------------------- Are there any other goals? ----------------------- */
            if (static_cast<int>(this->activities[this->ongoing_activity_ID].phase.size() - 1) > this->ongoing_phase_ID)
            {
                /* ---------------------------------- Debug --------------------------------- */
                std::cout << fmt::format("[Time]: {}", t) << std::endl;
                /* -------------------------------------------------------------------------- */


                /* ----- Are there any additional conditions that have to be satisfied? ----- */
                if(t >= this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID + 1].startTime && (this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID].switch_condition.first == false /*There aren't any additional conditions.*/|| this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID].switch_condition.second == true /*There is an additional condition, and it is satisfied.*/))
                {
                    /* ---------------------------- Current Phase ID ---------------------------- */
                    this->ongoing_phase_ID = this->ongoing_phase_ID + 1;
                    /* -------------------------------------------------------------------------- */

                    /* ------------------------------- Phase - Get ------------------------------ */    
                    phaseStruct& phase_ = this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID];
                    /* -------------------------------------------------------------------------- */

                    /* ------------------------------ Starting Time ----------------------------- */
                    double nominal_starting_time_ = this->activities[this->ongoing_activity_ID].nominal_start_time;

                    for(int j = 0; j < this->ongoing_phase_ID; j++)
                        nominal_starting_time_ = nominal_starting_time_ + this->activities[this->ongoing_activity_ID].phase[j].allocated_time;
                    /* -------------------------------------------------------------------------- */

                    /* ------------------- Goal - Construction and Allocation ------------------- */
                    double allocated_time = phase_.endTime - phase_.startTime;

                    if(allocated_time <= 10e-3 && phase_.type != -1)
                        phase_.type = -1;

                    phase_.goal = this->goal_generation(phase_.type,Eigen::Vector3d(this->position(0),this->position(1),this->theta),phase_.desired_configuration,allocated_time,t);
                    /* -------------------------------------------------------------------------- */

                    /* ------------------------------- Trajectory ------------------------------- */
                    if(phase_.goal.type != -1)
                        this->init_spline_trajectories(phase_.goal.initial_pose.head(2),phase_.goal.mid_pose.head(2),phase_.goal.final_pose.head(2),phase_.goal.times(0),phase_.goal.times(1),phase_.goal.times(2));
                    /* -------------------------------------------------------------------------- */

                    /* ------------------------------ Debug Message ----------------------------- */
                    std::cout << fmt::format("[{}][Robot-{}]: A New Goal has been allocated!",t,this->ID) << std::endl;
                    /* -------------------------------------------------------------------------- */

                }
                /* -------------------------------------------------------------------------- */
            }
            else
            {
                /* ---------------------------------- State --------------------------------- */
                this->ongoing_activity_ = false;
                this->activities[this->ongoing_activity_ID].state = activityStates::COMPLETED;
                /* -------------------------------------------------------------------------- */

                /* ------------------------------ Debug Message ----------------------------- */
                std::cout << fmt::format("[{}][Robot-{}]: The Ongoing Activity has been completed!", t, this->ID) << std::endl;
                /* -------------------------------------------------------------------------- */

                /* ------------------------------- Plan State ------------------------------- */
                if (static_cast<int>(this->activities.size() - 1) == this->ongoing_activity_ID)
                {
                    /* ------------------------------ Debug Message ----------------------------- */
                    std::cout << fmt::format("[{}][Robot-{}]: The processing of the given plan has been completed!", t, this->ID) << std::endl;
                    /* -------------------------------------------------------------------------- */

                    /* ---------------------------------- Reset --------------------------------- */
                    this->activities.clear();
                    this->ongoing_activity_ID = -1;
                    this->ongoing_phase_ID = -1;
                    /* -------------------------------------------------------------------------- */
                }
                /* -------------------------------------------------------------------------- */
            }
            /* -------------------------------------------------------------------------- */
        }
        /* -------------------------------------------------------------------------- */
    }
    else if(static_cast<int>(this->activities.size()) > 0 && static_cast<int>(this->activities.size() - 1) > this->ongoing_activity_ID && t >= this->activities[this->ongoing_activity_ID + 1].nominal_start_time)
    {
        /* ------------------------- Ongoing Activity State ------------------------- */
        this->ongoing_activity_ = true;
        /* -------------------------------------------------------------------------- */

        /* --------------------------- Ongoing Activity ID -------------------------- */
        this->ongoing_activity_ID = this->ongoing_activity_ID + 1;
        /* -------------------------------------------------------------------------- */

        /* -------------------------- State and Start Time -------------------------- */
        this->activities[this->ongoing_activity_ID].state = activityStates::STARTED;
        this->activities[this->ongoing_activity_ID].start_time = t;
        /* -------------------------------------------------------------------------- */

        /* ---------------------------- Phase Retrieving ---------------------------- */ 
        this->ongoing_phase_ID = 0;
        phaseStruct& phase_ = this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID];
        /* -------------------------------------------------------------------------- */

        /* ---------------------------- Goal Construction --------------------------- */
        double allocated_time = phase_.endTime - phase_.startTime;

        if(allocated_time <= 10e-3 && phase_.type != -1)
            phase_.type = -1;

        phase_.goal = this->goal_generation(phase_.type,Eigen::Vector3d(this->position(0),this->position(1),this->theta),phase_.desired_configuration,phase_.allocated_time,phase_.startTime);
        /* -------------------------------------------------------------------------- */

        /* ------------------------------- Trajectory ------------------------------- */
        this->init_spline_trajectories(phase_.goal.initial_pose.head(2),phase_.goal.mid_pose.head(2),phase_.goal.final_pose.head(2),phase_.goal.times(0),phase_.goal.times(1),phase_.goal.times(2));
        /* -------------------------------------------------------------------------- */

        /* ------------------------------ Debug Message ----------------------------- */
        std::cout << fmt::format("[{}][Robot-{}]: An Activity has been allocated!",t,this->ID) << std::endl;
        /* -------------------------------------------------------------------------- */
    }
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                               Goal Processing                              */
    /* -------------------------------------------------------------------------- */
    if(this->ongoing_activity_  && this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID].goal.type != -1)
    {
        /* ---------------------------------- Phase --------------------------------- */
        phaseStruct& phase_ = this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID];
        /* -------------------------------------------------------------------------- */

        /* ------------------------------- Trajectory ------------------------------- */
        this->evaluate_spline_and_derivatives(phase_.goal.initial_pose.head(2),phase_.goal.times(0),phase_.goal.final_pose.head(2),phase_.goal.times(2),t);
        /* -------------------------------------------------------------------------- */

        /* --------------------------------- Inputs --------------------------------- */
        this->robot_controller.y_1d = this->robot_controller.s(0);
        this->robot_controller.y_2d = this->robot_controller.s(1);

        this->robot_controller.dy_1d = this->robot_controller.ds(0);
        this->robot_controller.dy_2d = this->robot_controller.ds(1);

        u(0) = this->robot_controller.dy_1d + this->robot_controller.k_1*(this->robot_controller.y_1d - this->robot_controller.y_1);
        u(1) = this->robot_controller.dy_2d + this->robot_controller.k_2*(this->robot_controller.y_2d - this->robot_controller.y_2);
        /* -------------------------------------------------------------------------- */

        /* --------------------------- Obstacle Avoidance --------------------------- */
        if (this->robot_controller.obstacle_avoidance_enabled)
        {
            for(int i = 0; i < this->robot_controller.obstacles_position.size(); i++)
            {
                /* --------------------------- Auxiliary Variables -------------------------- */
                double distance_ij;
                Eigen::Vector2d r_ij;
                Eigen::Vector2d velocity_offsets;
                /* -------------------------------------------------------------------------- */

                /* ------------------------ Variables Initialization ------------------------ */
                this->robot_controller.avoidance_velocities.setZero();
                /* -------------------------------------------------------------------------- */

                /* -------------------------------- Distance -------------------------------- */
                distance_ij = (this->robot_controller.obstacles_position[i] - this->get_control_point_position()).norm();
                /* -------------------------------------------------------------------------- */

                /* ------------------------------- Unit Vector ------------------------------ */
                r_ij = (this->robot_controller.obstacles_position[i] - this->get_control_point_position())/distance_ij;
                /* -------------------------------------------------------------------------- */

                /* ---------------------------- Velocity Offsets ---------------------------- */
                if(distance_ij <= this->robot_controller.obstacle_avoidance_threshold)
                    this->robot_controller.avoidance_velocities = (-r_ij)*(this->robot_controller.obstacle_avoidance_gain/(distance_ij*distance_ij))*std::pow((1/distance_ij - 1/this->robot_controller.obstacle_avoidance_threshold),(this->robot_controller.obstacle_avoidance_eta - 1));
                /* -------------------------------------------------------------------------- */

                /* --------------------------------- Update --------------------------------- */
                u = u + this->robot_controller.avoidance_velocities;
                /* -------------------------------------------------------------------------- */
            }
        }
        /* -------------------------------------------------------------------------- */

        /* -------------------------- Transformation Matrix ------------------------- */
        T.row(0) << cos(this->theta), sin(this->theta);
        T.row(1) << -sin(this->theta)/this->robot_controller.b, cos(this->theta)/this->robot_controller.b;
        /* -------------------------------------------------------------------------- */

        /* ------------------------------- Velocities ------------------------------- */
        velocities = T*u;
        /* -------------------------------------------------------------------------- */

        /* ---------------------------- Saturation - Pt-1 --------------------------- */
        this->linear_velocity = sign(velocities(0))*std::min(abs(velocities(0)),this->robot_controller.max_linear_velocity);
        this->angular_velocity = sign(velocities(1))*std::min(abs(velocities(1)),this->robot_controller.max_angular_velocity);
        /* -------------------------------------------------------------------------- */

        /* -------------------------- Transformation Matrix ------------------------- */
        T.row(0) << cos(this->theta), -sin(this->theta)*this->robot_controller.b;
        T.row(1) << sin(this->theta), cos(this->theta)*this->robot_controller.b;
        /* -------------------------------------------------------------------------- */

        /* ---------------------------- Saturation - Pt-2 --------------------------- */
        velocities = T*Eigen::Vector2d(this->linear_velocity,this->angular_velocity);

        this->robot_controller.dy_1 = velocities(0);
        this->robot_controller.dy_2 = velocities(1);
        /* -------------------------------------------------------------------------- */
    }
    else
    {
        /* ------------------------------- Velocities ------------------------------- */
        this->robot_controller.dy_1 = 0.0;
        this->robot_controller.dy_2 = 0.0;
        this->robot_controller.dy_1d = 0.0;
        this->robot_controller.dy_2d = 0.0;
        this->linear_velocity = 0.0;
        this->angular_velocity = 0.0;
        /* -------------------------------------------------------------------------- */
    }
    /* -------------------------------------------------------------------------- */

}

void RobotControllerClass::update_obstacles_position(std::vector<Eigen::Vector2d> obstacles_position_)
{
    /* -------------------------------------------------------------------------- */
    /*                             Auxiliary Variables                            */
    /* -------------------------------------------------------------------------- */
    bool debug_message;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                          Variables Initialization                          */
    /* -------------------------------------------------------------------------- */
    debug_message = false;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                 Processing                                 */
    /* -------------------------------------------------------------------------- */

    /* ---------------------------------- Clear --------------------------------- */
    this->robot_controller.obstacles_position.clear();
    /* -------------------------------------------------------------------------- */

    /* --------------------------------- Update --------------------------------- */
    this->robot_controller.obstacles_position.insert(this->robot_controller.obstacles_position.end(),obstacles_position_.begin(),obstacles_position_.end());
    /* -------------------------------------------------------------------------- */

    /* ------------------------------ Debug Message ----------------------------- */
    if(debug_message)
        for(int i = 0; i < this->robot_controller.obstacles_position.size(); i++)
            std::cout << fmt::format("[Robot-{},Obstacle-{}][Position]: {},{}",this->ID,i+1,this->robot_controller.obstacles_position[i](0),this->robot_controller.obstacles_position[i](1)) << std::endl;
    /* -------------------------------------------------------------------------- */
    
    /* -------------------------------------------------------------------------- */
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                 Set Methods                                */
/* -------------------------------------------------------------------------- */
void RobotControllerClass::set_name(std::string name_)
{
    /* ----------------------------------- Set ---------------------------------- */
    this->name = name_;
    /* -------------------------------------------------------------------------- */
}

void RobotControllerClass::set_ID(int ID_)
{
    /* ----------------------------------- Set ---------------------------------- */
    this->ID = ID_;
    /* -------------------------------------------------------------------------- */
}

void RobotControllerClass::set_home_configuration(Eigen::Vector2d home_configuration_)
{
    /* ----------------------------------- Set ---------------------------------- */
    this->home_configuration = home_configuration_;
    /* -------------------------------------------------------------------------- */
}

void RobotControllerClass::set_odometry_state(bool state_)
{
    /* ----------------------------------- Set ---------------------------------- */
    this->odometry_state = state_;
    /* -------------------------------------------------------------------------- */
}

void RobotControllerClass::set_pose(Eigen::Vector2d position_, double theta_)
{
    /* ----------------------------------- Set ---------------------------------- */
    this->position = position_;
    this->theta = theta_;
    /* -------------------------------------------------------------------------- */

    /* ---------------------- Update Control Point Position --------------------- */
    this->robot_controller.y_1 = this->position(0) + (this->robot_controller.b * cos(this->theta));
    this->robot_controller.y_2 = this->position(1) + (this->robot_controller.b * sin(this->theta));
    /* -------------------------------------------------------------------------- */
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                 Get Methods                                */
/* -------------------------------------------------------------------------- */
std::string RobotControllerClass::get_name()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->name;
    /* -------------------------------------------------------------------------- */
}

int RobotControllerClass::get_ID()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->ID;
    /* -------------------------------------------------------------------------- */
}

int RobotControllerClass::get_ongoing_activity_ID()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->ongoing_activity_ID;
    /* -------------------------------------------------------------------------- */
}

int RobotControllerClass::get_ongoing_phase_ID()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->ongoing_phase_ID;
    /* -------------------------------------------------------------------------- */
}

Eigen::Vector2d RobotControllerClass::get_home_configuration()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->home_configuration;
    /* -------------------------------------------------------------------------- */
}

bool RobotControllerClass::get_odometry_state()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->odometry_state;
    /* -------------------------------------------------------------------------- */
}

Eigen::Vector2d RobotControllerClass::get_position()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->position;
    /* -------------------------------------------------------------------------- */
}

double RobotControllerClass::get_orientation()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->theta;
    /* -------------------------------------------------------------------------- */
}

double RobotControllerClass::get_linear_velocity()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->linear_velocity;
    /* -------------------------------------------------------------------------- */
}

double RobotControllerClass::get_angular_velocity()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->angular_velocity;
    /* -------------------------------------------------------------------------- */
}

Eigen::Vector2d RobotControllerClass::get_control_point_position()
{
    /* ----------------------------------- Get ---------------------------------- */
    return Eigen::Vector2d(this->robot_controller.y_1,this->robot_controller.y_2);
    /* -------------------------------------------------------------------------- */
}

Eigen::Vector2d RobotControllerClass::get_control_point_velocity()
{
    /* ----------------------------------- Get ---------------------------------- */
    return Eigen::Vector2d(this->robot_controller.dy_1,this->robot_controller.dy_2);
    /* -------------------------------------------------------------------------- */
}

Eigen::Vector2d RobotControllerClass::get_desired_control_point_position()
{
    /* ----------------------------------- Get ---------------------------------- */
    return Eigen::Vector2d(this->robot_controller.y_1d,this->robot_controller.y_2d);
    /* -------------------------------------------------------------------------- */
}

Eigen::Vector2d RobotControllerClass::get_desired_control_point_velocity()
{
    /* ----------------------------------- Get ---------------------------------- */
    return Eigen::Vector2d(this->robot_controller.dy_1d,this->robot_controller.dy_2d);
    /* -------------------------------------------------------------------------- */
}

Eigen::Vector2d RobotControllerClass::get_avoidance_velocity_offsets()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->robot_controller.avoidance_velocities;
    /* -------------------------------------------------------------------------- */
}

double RobotControllerClass::get_control_error()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->robot_controller.error;
    /* -------------------------------------------------------------------------- */
}

ongoingActivityStruct RobotControllerClass::get_ongoing_activity()
{
    /* -------------------------------------------------------------------------- */
    /*                             Auxiliary Variables                            */
    /* -------------------------------------------------------------------------- */
    ongoingActivityStruct ongoing_activity_;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                          Variables Initialization                          */
    /* -------------------------------------------------------------------------- */

    /* ---------------------------------- Check --------------------------------- */
    if(this->ongoing_activity != nullptr)
        ongoing_activity_ = *(this->ongoing_activity);
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */

    /* ----------------------------------- Get ---------------------------------- */
    return ongoing_activity_;
    /* -------------------------------------------------------------------------- */
}

std::vector<activityStruct> RobotControllerClass::get_activities()
{
    /* ----------------------------------- Get ---------------------------------- */
    return this->activities;
    /* -------------------------------------------------------------------------- */
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                Other Methods                               */
/* -------------------------------------------------------------------------- */
void RobotControllerClass::allocate_a_plan(std::vector<activityStruct> activities_)
{
    /* -------------------------------------------------------------------------- */
    /*                             Auxiliary Variables                            */
    /* -------------------------------------------------------------------------- */

    /* -------------------- Ongoing Activity/Phase Components ------------------- */
    phaseStruct phase_;
    goalStruct goal_;
    double time_;
    Eigen::Vector3d initial_pose_contact;
    Eigen::Vector3d final_pose_contact;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                          Variables Initialization                          */
    /* -------------------------------------------------------------------------- */
    
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                Re-Allocation                               */
    /* -------------------------------------------------------------------------- */
    if(static_cast<int>(this->activities.size()) > 0)
    {
        /* ---------------------------------- Loop ---------------------------------- */
        for(int i = 0; i < this->activities.size(); i++)
        {
            /* ---------------------------------- Check --------------------------------- */
            if(this->activities[i].state == activityStates::NOT_STARTED_YET)
                this->activities[i] = activities_[i];
            else if(this->activities[i].state == activityStates::STARTED)
            {
                /* ------------------- Activity Update - Start Time Update ------------------ */
                this->activities[i].nominal_start_time = activities_[i].nominal_start_time;
                /* -------------------------------------------------------------------------- */

                /* -------------------- Activity Update - Phases Updating ------------------- */
                for(int j = 0; j < activities_[i].phase.size(); j++)
                {
                    /* ---------------------------------- Type ---------------------------------- */
                    this->activities[i].phase[j].type = activities_[i].phase[j].type;
                    /* -------------------------------------------------------------------------- */

                    /* ---------------------------------- Time ---------------------------------- */
                    this->activities[i].phase[j].startTime = activities_[i].phase[j].startTime;
                    this->activities[i].phase[j].endTime = activities_[i].phase[j].endTime;
                    this->activities[i].phase[j].allocated_time = activities_[i].phase[j].allocated_time;
                    /* -------------------------------------------------------------------------- */

                    /* ------------------------------ Configuration ----------------------------- */
                    this->activities[i].phase[j].desired_configuration = activities_[i].phase[j].desired_configuration;
                    /* -------------------------------------------------------------------------- */

                    /* -------------------------------- Condition ------------------------------- */
                    this->activities[i].phase[j].switch_condition = activities_[i].phase[j].switch_condition;
                    /* -------------------------------------------------------------------------- */
                }
                /* -------------------------------------------------------------------------- */

                /* ---------------------- Initial Time of the Activity ---------------------- */
                time_ = this->activities[i].nominal_start_time;
                /* -------------------------------------------------------------------------- */

                /* -------------------- Initial time of the Current Phase ------------------- */
                for(int j = 0; j < this->ongoing_phase_ID; j++)
                    time_ = time_ + this->activities[i].phase[j].allocated_time;
                /* -------------------------------------------------------------------------- */

                /* ------------------------- From Control to Contact ------------------------ */
                initial_pose_contact = control2contact(this->activities[i].phase[this->ongoing_phase_ID].goal.initial_pose /*Initial Pose of the Control Point*/, this->robot_controller.b /*Displacement wrt the contact point*/);
                final_pose_contact = this->activities[i].phase[this->ongoing_phase_ID].desired_configuration;
                /* -------------------------------------------------------------------------- */


                /* --------- Re-generation of the last goal of the ongoing activity --------- */
                this->activities[i].phase[this->ongoing_phase_ID].goal = this->goal_generation(this->activities[i].phase[this->ongoing_phase_ID].type /*Type of activity*/,initial_pose_contact /*Initial Pose of the Contact Point*/,final_pose_contact /*Final Pose desired for the Contact Point*/,this->activities[i].phase[this->ongoing_phase_ID].allocated_time /*Time available for performing the task*/,time_ /*Initial Time*/);
                /* -------------------------------------------------------------------------- */

                /* --------------------------- Trajectory Updating -------------------------- */
                this->init_spline_trajectories(this->activities[i].phase[this->ongoing_phase_ID].goal.initial_pose.head(2) /*Initial Position of the Control Point*/,this->activities[i].phase[this->ongoing_phase_ID].goal.mid_pose.head(2) /*Mid Position desired for the Control Point*/,this->activities[i].phase[this->ongoing_phase_ID].goal.final_pose.head(2) /*Final Position desired for the Contact Point*/,this->activities[i].phase[this->ongoing_phase_ID].goal.times(0) /*Initial Time*/,this->activities[i].phase[this->ongoing_phase_ID].goal.times(1) /*The Control Point should reach the mid position at this time*/,this->activities[i].phase[this->ongoing_phase_ID].goal.times(2) /*The Control Point should reach the final position at this time*/);
                /* -------------------------------------------------------------------------- */

            }
            /* -------------------------------------------------------------------------- */
        }   
        /* -------------------------------------------------------------------------- */
    }
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                 Allocation                                 */
    /* -------------------------------------------------------------------------- */
    else
        /* ------------------------------- Allocation ------------------------------- */
        this->activities.insert(this->activities.end(),activities_.begin(),activities_.end());
        /* -------------------------------------------------------------------------- */
        
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
}

goalStruct RobotControllerClass::goal_generation(int type_, Eigen::Vector3d initial_pose_, Eigen::Vector3d final_pose_, double allocated_time_, double starting_time_)
{
    /* -------------------------------------------------------------------------- */
    /*                             Auxiliary Variables                            */
    /* -------------------------------------------------------------------------- */
    goalStruct goal_;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                          Variables Initialization                          */
    /* -------------------------------------------------------------------------- */
    goal_.type = type_;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                 Processing                                 */
    /* -------------------------------------------------------------------------- */

    /* ------------------------------ Moving Phase ------------------------------ */
    if(type_ != -1)
    {
        /* ------------------------------ Initial Pose ------------------------------ */
        goal_.initial_pose(2) = initial_pose_(2);
        goal_.initial_pose(0) = initial_pose_(0) + (this->robot_controller.b * cos(initial_pose_(2)));
        goal_.initial_pose(1) = initial_pose_(1) + (this->robot_controller.b * sin(initial_pose_(2)));
        /* -------------------------------------------------------------------------- */

        /* ------------------------------- Final Pose ------------------------------- */
        goal_.final_pose(2) = final_pose_(2);
        goal_.final_pose(0) = final_pose_(0) + (this->robot_controller.b * cos(final_pose_(2)));
        goal_.final_pose(1) = final_pose_(1) + (this->robot_controller.b * sin(final_pose_(2)));
        /* -------------------------------------------------------------------------- */

        /* -------------------------------- Mid Pose -------------------------------- */
        if(type_ == 2)
        {
            /* ------------------------------- Orientation ------------------------------ */
            goal_.mid_pose(2) = final_pose_(2);
            /* -------------------------------------------------------------------------- */

            /* -------------------------------- Position -------------------------------- */
            goal_.mid_pose.head(2) = (goal_.initial_pose.head(2) + goal_.final_pose.head(2))/2.0;
            /* -------------------------------------------------------------------------- */
        }
        else
        {
            /* --------------------------- Corner Orientation --------------------------- */
            goal_.mid_pose(2) = (initial_pose_(2) + final_pose_(2))/2.0;
            /* -------------------------------------------------------------------------- */

            /* ----------------------------- Corner Position ---------------------------- */
            if(type_ == 0)
            {
                /* ------------------------------------ X ----------------------------------- */
                goal_.mid_pose(0) = final_pose_(0) + (this->robot_controller.b * cos(goal_.mid_pose(2)));
                /* -------------------------------------------------------------------------- */
                /* ------------------------------------ Y ----------------------------------- */
                goal_.mid_pose(1) = initial_pose_(1) + (this->robot_controller.b * sin(goal_.mid_pose(2)));
                /* -------------------------------------------------------------------------- */
            }
            else
            {
                /* ------------------------------------ X ----------------------------------- */
                goal_.mid_pose(0) = initial_pose_(0) + (this->robot_controller.b * cos(goal_.mid_pose(2)));
                /* -------------------------------------------------------------------------- */
                /* ------------------------------------ Y ----------------------------------- */
                goal_.mid_pose(1) = final_pose_(1) + (this->robot_controller.b * sin(goal_.mid_pose(2)));
                /* -------------------------------------------------------------------------- */
            }
            /* -------------------------------------------------------------------------- */
        }
        /* -------------------------------------------------------------------------- */

        /* -------------------------------- Distances ------------------------------- */
        double d_ic = (goal_.mid_pose.head(2) - goal_.initial_pose.head(2)).norm();
        double d_cf = (goal_.final_pose.head(2) - goal_.mid_pose.head(2)).norm();
        double d = d_ic + d_cf;
        /* -------------------------------------------------------------------------- */

        /* ---------------------------------- Times --------------------------------- */
        goal_.times(0) = starting_time_;
        goal_.times(1) = goal_.times(0) + (allocated_time_*(d_ic/d));
        goal_.times(2) = goal_.times(1) + (allocated_time_*(d_cf/d));
        /* -------------------------------------------------------------------------- */

        /* ------------------------------ Displacement ------------------------------ */
        goal_.displacement = (goal_.final_pose.head(2) - goal_.final_pose.head(2)).norm();
        /* -------------------------------------------------------------------------- */
    }
    /* -------------------------------------------------------------------------- */
    /* -------------------------- Serving/Waiting Phase ------------------------- */
    else
    {
        /* ------------------------------ Initial Pose ------------------------------ */
        goal_.initial_pose(2) = initial_pose_(2);
        goal_.initial_pose(0) = initial_pose_(0) + (this->robot_controller.b * cos(initial_pose_(2)));
        goal_.initial_pose(1) = initial_pose_(1) + (this->robot_controller.b * sin(initial_pose_(2)));
        /* -------------------------------------------------------------------------- */

        /* ------------------------------- Corner Pose ------------------------------ */
        goal_.mid_pose.setConstant(-1);
        /* -------------------------------------------------------------------------- */

        /* ------------------------------- Final Pose ------------------------------- */
        goal_.final_pose = goal_.initial_pose;
        /* -------------------------------------------------------------------------- */

        /* ---------------------------------- Times --------------------------------- */
        goal_.times(0) = starting_time_;
        goal_.times(1) = -1;
        goal_.times(2) = starting_time_ + allocated_time_;
        /* -------------------------------------------------------------------------- */

        /* ------------------------------ Displacement ------------------------------ */
        goal_.displacement = (goal_.final_pose.head(2) - goal_.final_pose.head(2)).norm();
        /* -------------------------------------------------------------------------- */
    }
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                    Debug                                   */
    /* -------------------------------------------------------------------------- */
    std::cout << fmt::format("[{}]: Start: {}, ComputedEnd: {}, NominalEnd: {}",this->activities[this->ongoing_activity_ID].phase[this->ongoing_phase_ID].name,goal_.times(0),goal_.times(2),goal_.times(0) + allocated_time_)  << std::endl;
    std::cout << "InitialPose: " << initial_pose_.transpose() << " FinalPose: " << final_pose_.transpose() << std::endl;
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                   Return                                   */
    /* -------------------------------------------------------------------------- */
    return goal_;
    /* -------------------------------------------------------------------------- */
}

void RobotControllerClass::evaluate_spline_and_derivatives(Eigen::Vector2d starting_position_, double starting_time_, Eigen::Vector2d final_position_, double final_time_, double time_)
{
    /* -------------------------------------------------------------------------- */
    /*                             Auxiliary Variables                            */
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                          Variables Initialization                          */
    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */

    /* -------------------------------------------------------------------------- */
    /*                                 Processing                                 */
    /* -------------------------------------------------------------------------- */
    if(time_ < starting_time_)
    {
        /* -------------------------------- Position -------------------------------- */
        this->robot_controller.s = starting_position_;
        /* -------------------------------------------------------------------------- */

        /* -------------------------------- Velocity -------------------------------- */
        this->robot_controller.ds.setZero();
        /* -------------------------------------------------------------------------- */
    }
    else if(time_ >= starting_time_ && time_ <= final_time_)
    {
        /* -------------------------- Position and Velocity ------------------------- */
        spline1ddiff(this->robot_controller.traj_x,time_,this->robot_controller.s(0),this->robot_controller.ds(0),this->robot_controller.dds(0));
        spline1ddiff(this->robot_controller.traj_y,time_,this->robot_controller.s(1),this->robot_controller.ds(1),this->robot_controller.dds(1));
        /* -------------------------------------------------------------------------- */
    }
    else
    {
        /* -------------------------------- Position -------------------------------- */
        this->robot_controller.s = final_position_;
        /* -------------------------------------------------------------------------- */

        /* -------------------------------- Velocity -------------------------------- */
        this->robot_controller.ds.setZero();
        /* -------------------------------------------------------------------------- */
    }
    /* -------------------------------------------------------------------------- */
}
/* -------------------------------------------------------------------------- */