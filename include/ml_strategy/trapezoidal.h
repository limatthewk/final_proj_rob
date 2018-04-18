#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <ros/ros.h>


// Trapezoidal profile methods
namespace trapezoidal {

// Estimate minimum time that a vehicle takes to start from 'p0',
// reach 'pf' with maximum speed possible, with upper bounds on 
// maximum velocity 'v_max' and acceleration 'a_max'
void estimate_final_time(const double &p0,
	                     const double &pf,
	                     const double &v0,
	                     const double &v_max,
	                     const double &a_max,
	                     double *tf) {
	// Velocity profile (if v0 >= 0)
	//
	// v_max|   ___________________
	//      |  /'                 '
	//      | / '                 '
	//    v0|/  '                 '
	//      |   '                 '
	//      ----'-----------------'---------
	//      0   t1                tf

	// Velocity profile (if v0 < 0)
	//
	// v_max|        ___________________
	//      |       /'                 '
	//      |      / '                 '
	//      |     /  '                 '
	//      |    /   '                 '
	//      ----/----'-----------------'---------
	//     0|  /      t1                tf
	//      | /
	//    v0|/

	// S1 - displacement until reaching t1
	// S2 - displacement between t1 and tf
	// S - total displacement


	// Assume only strictly positive accelerations
	if(a_max <= 0) {
		*tf = std::numeric_limits<double>::infinity();
		ROS_WARN("Negative acceleration!");
		return;
	}

	// We only assume positive displacements
	const double S = pf - p0;
	if (S <= 0) {
		*tf = 0;
		ROS_WARN("Negative displacement!");
		return;
	}
	
	// Formulas valid for v0 >= 0, and v0 < 0
	const double t1 = (v_max - v0)/a_max;
	const double S1 = t1*(v_max + v0)/2.0;

	if(S < S1) {
		if (v0 >= 0) {
			const double vf = sqrt(v0*v0 + 2*a_max*S);
			*tf = 2*S/(v0 + vf);
		} else {  // v0 < 0
			const double A1 = -v0*v0/(2*a_max);  // Area below the time axis
			const double A2 = S - A1;			 // Area above time axis
			const double vf = sqrt(2*a_max*A2);
			const double t0 = -v0/a_max;
			*tf = t0 + vf/a_max;
		}
	} else {  // S >=S1
		const double S2 = S - S1;
		const double t1 = (v_max - v0)/a_max;
		*tf = t1 + S2/v_max;
	}

}

}  // namespace trapezoidal

#endif  // STRUCTS_H_