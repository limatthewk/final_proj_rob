#ifndef STRATEGY_CLASS_H_
#define STRATEGY_CLASS_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "ml_strategy/helper.h"
#include "ml_strategy/trapezoidal.h"
#include "ml_strategy/linear_algebra.h"
#include "ml_strategy/rk4.h"
#include "nav_msgs/Odometry.h"
#include "mg_msgs/PVA.h"

// Offensive states
class AttackStates{
 public:
	uint RETURNING = 0;  // Return to team's area
	uint ADVANCING = 1;  // Going towards opponent's team base
	uint BALLOON = 2;    // Aiming the balloon
    //uint STUCKED=3;  //TEST

    uint BACK=4;//back and attack

	uint State = ADVANCING;
};
// Defensive states
class DefenseStates{
 public:
	uint STEADY = 0;  	 // Steady at a given area
	uint TARGETING = 1;  // Targeting an enemy
	uint RETURNING = 2;  // Returning home?
    uint FORWARD=3;
	uint State = STEADY;
	std::string target_name = "";
};

// Quadcopter roles
class QuadRole{
 public:
	uint GOALKEEPER = 0;
	uint DEFENSIVE_RIGHT = 1;
	uint DEFENSIVE_LEFT = 2;
	uint DEFENSIVE_CENTRAL = 3;
	uint OFFENSIVE_RIGHT = 4;
	uint OFFENSIVE_LEFT = 5;
	uint OFFENSIVE_CENTRAL = 6;

	uint State = GOALKEEPER;
	AttackStates AttackState;
	DefenseStates DefenseState;
};

// Enemy danger assignment
class EnemyDanger{
 public:
	uint NEUTRAL = 0;
	uint WARNING = 1;
	uint DANGER = 2;

	uint State = NEUTRAL;
};

class QuadState{
 public:
    Eigen::Vector3d position;            // Quad position (meters)
    Eigen::Quaterniond orientation;      // Quad orientation (quaternion)
    Eigen::Vector3d velocity;            // Quad velocity (m/s)
    Eigen::Vector3d angular_velocity;    // Quad angular velocity (rad/s)
};

class QuadData {
 public:
    std::string name;                            // Unique name for vehicle
    mutable QuadState quad_state;                // Measured quad states
    mutable QuadRole role;					     // Quad role in the team

    mutable mg_msgs::PVA reference;			     // Output reference structure
    mutable Eigen::Vector3d init_pos;			 // Initial pos to return later on (meters)
    mutable rk4 reference_integrator;            // Runge-kutta dynamics integrator
    mutable ros::NodeHandle nh;                  // ROS Nodehandle
    mutable ros::Publisher pub_reference;  		 // Publishes the reference

    bool operator<(const QuadData& other) const {
        int compareResult = name.compare(other.name);
        return (compareResult < 0);
    }
};

class EnemyData {
 public:
    std::string name;                            // Unique name for vehicle
    mutable QuadState quad_state;                // Measured quad states
    mutable EnemyDanger danger;					 // Danger labels for enemies
    mutable bool targeted;						 // Enemy is being targeted by a teammate

    bool operator<(const EnemyData& other) const {
        int compareResult = name.compare(other.name);
        return (compareResult < 0);
    }
};

class TeamStrategy {
 public:
 	std::set<QuadData> quads_;
 	std::set<EnemyData> enemies_;
 	Eigen::Vector3d team_balloon_, enemy_balloon_;
 	uint n_quads_, n_enemies_;
    double max_acc_, max_vel_;
    Eigen::Vector3d offensive_direction_ = Eigen::Vector3d( 1.0, 0.0, 0.0);
    Eigen::Vector3d defensive_direction_ = Eigen::Vector3d(-1.0, 0.0, 0.0);
    Plane3d enemy_balloon_plane_;  // Used to find distance to balloon plane
    bool balloon_targeted_ = false;
    bool balloon_popped_ = false;

 	// Constructors
 	TeamStrategy();
 	TeamStrategy(const double max_vel,
 		         const double max_acc,
 		         const Eigen::Vector3d team_balloon,
 		         const Eigen::Vector3d enemy_balloon);

 	// Methods
    void PrintQuadNames();
    void PrintQuadReferences(const std::string &name);
    void AddQuad(const std::string &quad_name,
	 		     const uint &role,
	 		     const Eigen::Vector3d &ref_pos,
	 		     const double &yaw_ref,
		         const std::string &output_topic,
		         ros::NodeHandle *nh);
	void AddEnemy(const std::string &enemy_name,
			      const nav_msgs::Odometry &odom);
    void Odom2QuatStates(const nav_msgs::Odometry &odom,
                         QuadState *quad_state);
    void UpdateQuadOdom(const std::string &name, 
                        const nav_msgs::Odometry &odom);
    void FindQuadIndex(const std::string &name,
    	               std::set<QuadData>::iterator *index);  // Returns -1 if it can't find
	void FindEnemyIndex(const std::string &name,
                		std::set<EnemyData>::iterator *index);
    void PublishReferences();

    // Functions to update enemy danger
    void EnemyDangerUpdate();
    void GetDangerousEnemies(std::vector<std::string> *names,
                             std::vector<std::set<EnemyData>::iterator> *iterators);

    // Rules to update defensive and offensive state machines
    void UpdateAttDefStateMachine();
    void UpdateOffensive(const std::set<QuadData>::iterator &it,
                       const std::set<EnemyData>::iterator &itEnemies);
    void UpdateDefensive(const std::set<QuadData>::iterator &it);

    // Strategy-dependent methods
    void UpdateReferences(const double &dt);
    mg_msgs::PVA GetRefRk4(const std::set<QuadData>::iterator &it,
	                       const double &dt);
	void OffensiveReturn(const std::set<QuadData>::iterator &it,
	                     const double &dt);
    void OffensiveAdvance(const std::set<QuadData>::iterator &it,
	                      const double &dt);
    void OffensiveBalloon(const std::set<QuadData>::iterator &it,
	                      const double &dt);

    //-------
    void DefensiveForward(const std::set<QuadData>::iterator &it,
                         const double &dt);
//----------------
    void DefensiveSteady(const std::set<QuadData>::iterator &it,
	                     const double &dt);
    void DefensiveTargeting(const std::set<QuadData>::iterator &it,
	                        const double &dt);
	void DefensiveReturn(const std::set<QuadData>::iterator &it,
	                     const double &dt);


};

#endif  // STRATEGY_CLASS_H_