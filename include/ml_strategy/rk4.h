

#ifndef RK4_H_
#define RK4_H_

#include <Eigen/Dense>

// ROS message types
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "mg_msgs/PVA.h"
#include "ml_strategy/helper.h"

class rk4 {  // Runge-kutta for time-invariant systems
 public:
    Eigen::Vector3d y_;      // Integrated position
    Eigen::Vector3d y_dot_;  // Integrated velocity
    Eigen::Vector3d y_ddot_; // Acceleration
    Eigen::Matrix3d K_;
    Eigen::Matrix3d Kd_;
    double max_acc_;
    double max_vel_;
    
 	// Constructors
 	rk4();
 	rk4(const double &max_vel,
        const double &max_acc,
        const Eigen::Vector3d &Pos);

 	// Methods
	void DifferentialEquation(const Eigen::Vector3d &F,
						      const Eigen::VectorXd &state0,
	                          Eigen::VectorXd *state_dot);
 	void UpdateStates(const Eigen::Vector3d &F,
                  	  const double &dt);
    void ResetStates(const Eigen::Vector3d &Pos);
    void ResetStates(const nav_msgs::Odometry &odom);
    void SetPos(const Eigen::Vector3d &pos);
    void GetPos(Eigen::Vector3d *pos);
    void GetPos(geometry_msgs::Point *pos);
    void GetVel(Eigen::Vector3d *vel);
    void GetVel(geometry_msgs::Vector3 *vel);
    void GetAcc(Eigen::Vector3d *acc);
    void GetAcc(geometry_msgs::Vector3 *acc);

 private:
};


#endif  // ML_CLASS_H_