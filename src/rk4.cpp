#include "ml_strategy/rk4.h"

// Runge-kutta class ----------------------------------------------
rk4::rk4() {

}

rk4::rk4(const double &max_vel,
	     const double &max_acc,
         const Eigen::Vector3d &Pos) {
	max_acc_ = max_acc;
	max_vel_ = max_vel;
	y_ = Pos;
	y_dot_ = Eigen::Vector3d::Zero();
	y_ddot_ = Eigen::Vector3d::Zero();
	// std::cout << "A:\n" << A << std::endl;
	// std::cout << "B:\n" << B << std::endl;
}

void rk4::DifferentialEquation(const Eigen::Vector3d &F,
							   const Eigen::VectorXd &state0,
	                           Eigen::VectorXd *state_dot) {
	// Get references
	Eigen::Vector3d yd, yd_dot, yd_ddot;
	// yd << Ref.Pos.x, Ref.Pos.y, Ref.Pos.z;
	// yd_dot << Ref.Vel.x, Ref.Vel.y, Ref.Vel.z;
	// yd_ddot << Ref.Acc.x, Ref.Acc.y, Ref.Acc.z;

	// Differential equation
	// Eigen::Vector3d y, y_dot, y_ddot, e, e_dot;
	// y << state0[0], state0[1], state0[2];
	// y_dot << state0[3], state0[4], state0[5];
	// e = yd - y;
	// e_dot = yd_dot - y_dot;
	// y_ddot = yd_ddot + K_*e + Kd_*e_dot + F;
	Eigen::Vector3d y_ddot, y_dot, y;
	y << state0[0], state0[1], state0[2];
	y_dot << state0[3], state0[4], state0[5];
	y_ddot = F;

	// Cap acceleration to maximum value
	if(y_ddot.norm() > max_acc_) {
		y_ddot = max_acc_*y_ddot.normalized();
	}

	// If velocity is above norm, we only allow accelerations that
	// will reduce the velocity (or rotate it)
	if(y_dot.norm() >= max_vel_) {
		// Project acceleration into velocity
		const double projection = y_ddot.dot(y_dot.normalized());

		// If projection is positive, we only allow y_ddot to rotate velocity
		if(projection > 0) {
			y_ddot = y_ddot - projection*y_dot.normalized();  // Remove the component along vel
		}
	}

	Eigen::VectorXd x_dot(6);
	x_dot << y_dot[0], y_dot[1], y_dot[2], y_ddot[0], y_ddot[1], y_ddot[2];

	*state_dot = x_dot;
}

void rk4::UpdateStates(const Eigen::Vector3d &F,
	          	       const double &dt) {
	const double dt_half = dt/2.0;
	Eigen::VectorXd k1(6), k2(6), k3(6), k4(6);

	Eigen::VectorXd state(6);
	state << y_[0], y_[1], y_[2], y_dot_[0], y_dot_[1], y_dot_[2];

	this->DifferentialEquation(F, state, &k1);
	this->DifferentialEquation(F, state + dt_half*k1, &k2);
	this->DifferentialEquation(F, state + dt_half*k2, &k3);
	this->DifferentialEquation(F, state + dt*k3, &k4);

	Eigen::VectorXd deltaX = dt*(k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
	state = state + deltaX;
	y_ << state[0], state[1], state[2];
	y_dot_ << state[3], state[4], state[5];
	y_ddot_ = Eigen::Vector3d(k1[3], k1[4], k1[5]);
}

void rk4::ResetStates(const Eigen::Vector3d &Pos) {
	y_ = Pos;
	y_dot_ << 0.0, 0.0, 0.0;
	y_ddot_ << 0.0, 0.0, 0.0;
}

void rk4::ResetStates(const nav_msgs::Odometry &odom) {
	y_ << odom.pose.pose.position.x,
	      odom.pose.pose.position.y,
	      odom.pose.pose.position.z;
	y_dot_ << 0.0, 0.0, 0.0;
	y_ddot_ << 0.0, 0.0, 0.0;	
}

void rk4::SetPos(const Eigen::Vector3d &pos) {
	y_ = pos;
}

void rk4::GetPos(Eigen::Vector3d *pos) {
	// *pos = Eigen::Vector3d(state_[0], state_[1], state_[2]);
	*pos = y_;
	// std::cout << y_[0] << " " << y_[1] << " " << y_[2] << std::endl;
}

void rk4::GetPos(geometry_msgs::Point *pos) {
	Eigen::Vector3d eigen_pos;
	this->GetPos(&eigen_pos);
	pos->x = eigen_pos[0];
	pos->y = eigen_pos[1];
	pos->z = eigen_pos[2];
}

void rk4::GetVel(Eigen::Vector3d *vel) {
	// *vel = Eigen::Vector3d(state_[3], state_[4], state_[5]);
	*vel = y_dot_;
}

void rk4::GetVel(geometry_msgs::Vector3 *vel) {
	Eigen::Vector3d eigen_vel;
	this->GetVel(&eigen_vel);
	vel->x = eigen_vel[0];
	vel->y = eigen_vel[1];
	vel->z = eigen_vel[2];
}

void rk4::GetAcc(Eigen::Vector3d *acc) {
	// *acc = accel_;
	*acc = y_ddot_;
}

void rk4::GetAcc(geometry_msgs::Vector3 *acc) {
	Eigen::Vector3d eigen_acc;
	this->GetAcc(&eigen_acc);
	acc->x = eigen_acc[0];
	acc->y = eigen_acc[1];
	acc->z = eigen_acc[2];
}