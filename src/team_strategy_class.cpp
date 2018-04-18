#include "ml_strategy/team_strategy_class.h"

TeamStrategy::TeamStrategy() {
	n_quads_ = 0;
	n_enemies_ = 0;
}

TeamStrategy::TeamStrategy(const double max_vel,
 	                       const double max_acc,
		 		           const Eigen::Vector3d team_balloon,
		 		           const Eigen::Vector3d enemy_balloon) {
	max_vel_ = max_vel;
	max_acc_ = max_acc;
	team_balloon_ = team_balloon;
	enemy_balloon_ = enemy_balloon;
	offensive_direction_ = (enemy_balloon - team_balloon).normalized();
	defensive_direction_ = -offensive_direction_;
	enemy_balloon_plane_ = Plane3d(enemy_balloon_, defensive_direction_);
	n_quads_ = 0;
	n_enemies_ = 0;
}

// Methods
void TeamStrategy::PrintQuadNames() {
	std::set<QuadData>::iterator it1;
	std::cout << "Team quads:" << std::endl;
	for(it1 = quads_.begin(); it1 != quads_.end(); ++it1) {
		std::cout << it1->name << std::endl;
	}
	std::set<EnemyData>::iterator it2;
	std::cout << "Enemy quads:" << std::endl;
	for(it2 = enemies_.begin(); it2 != enemies_.end(); ++it2) {
		std::cout << it2->name << std::endl;
	}
	std::cout << std::endl;
}

void TeamStrategy::PrintQuadReferences(const std::string &name) {
	std::set<QuadData>::iterator it;
	this->FindQuadIndex(name, &it);
	if (it != quads_.end()) {
		ROS_INFO("Quad %s references: %f\t%f\t%f", name.c_str(), 
			     it->reference.Pos.x, it->reference.Pos.y, it->reference.Pos.z);
	} else {
		ROS_INFO("[mediation layer] Couldn't print reference: quad name not found");
	}
}

void TeamStrategy::AddQuad(const std::string &quad_name,
    			 		   const uint &role,
    			 		   const Eigen::Vector3d &ref_pos,
    			 		   const double &yaw_ref,
					       const std::string &output_topic,
					       ros::NodeHandle *nh) {
	QuadData new_quad;
	new_quad.name = quad_name;

	// Check whether quad name already exists
	if(quads_.find(new_quad) != quads_.end()) {
		ROS_WARN("[mediation layer] Tried to add quad ""%s"": already exists!", quad_name.c_str());
	} else {
		mg_msgs::PVA emptyPVA = helper::GetEmptyPVA();
		new_quad.reference = emptyPVA;
		new_quad.reference.Pos = helper::Vec3d2point(ref_pos);
		new_quad.reference.yaw = yaw_ref;
		new_quad.init_pos = ref_pos;
		new_quad.quad_state.position = ref_pos;
		new_quad.role.State = role;
		new_quad.reference_integrator = rk4(max_vel_, max_acc_, ref_pos);
		new_quad.nh = *nh;
		new_quad.pub_reference = new_quad.nh.advertise<mg_msgs::PVA>(output_topic, 1);
		quads_.insert(new_quad);
		n_quads_ = n_quads_ + 1;
	}
}

void TeamStrategy::AddEnemy(const std::string &enemy_name,
		      				const nav_msgs::Odometry &odom) {
	EnemyData new_enemy;
	new_enemy.name = enemy_name;

	// Check whether enemy name already exists
	if(enemies_.find(new_enemy) != enemies_.end()) {
		ROS_WARN("[mediation layer] Tried to add quad ""%s"": already exists!", enemy_name.c_str());
	} else {
		new_enemy.targeted = false;
		enemies_.insert(new_enemy);
		n_enemies_ = n_enemies_ + 1;
	}
}

void TeamStrategy::Odom2QuatStates(const nav_msgs::Odometry &odom,
                                   QuadState *quad_state) {
	quad_state->position = helper::Point2vec3d(odom.pose.pose.position);
	quad_state->orientation = helper::RosQuat2EigenQuat(odom.pose.pose.orientation);
	quad_state->velocity = helper::Vec32vec3d(odom.twist.twist.linear);
	quad_state->angular_velocity = helper::Vec32vec3d(odom.twist.twist.angular);
}

void TeamStrategy::UpdateQuadOdom(const std::string &name, 
                                  const nav_msgs::Odometry &odom) {
	std::set<QuadData>::iterator it1;
	this->FindQuadIndex(name, &it1);
	if (it1 != quads_.end()) {  // The quad is in the team
		this->Odom2QuatStates(odom, &it1->quad_state);
	} else {  // If not in the team, must be an enemy
		std::set<EnemyData>::iterator it2;
		this->FindEnemyIndex(name, &it2);
		if (it2 != enemies_.end()) {  // If enemy exists
			this->Odom2QuatStates(odom, &it2->quad_state);
		} else {  // Create new enemy
			this->AddEnemy(name, odom);
		}
	}
}

void TeamStrategy::FindQuadIndex(const std::string &name,
               					 std::set<QuadData>::iterator *index){
	QuadData quad_with_name;
	quad_with_name.name = name;
	*index = quads_.find(quad_with_name);
}


void TeamStrategy::FindEnemyIndex(const std::string &name,
               					  std::set<EnemyData>::iterator *index){
	EnemyData quad_with_name;
	quad_with_name.name = name;
	*index = enemies_.find(quad_with_name);
}

void TeamStrategy::PublishReferences() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		it->pub_reference.publish(it->reference);
		// ROS_INFO("%s: %f %f %f", it->name.c_str(), it->reference.Pos.x, it->reference.Pos.y, it->reference.Pos.z);
	}
}

void TeamStrategy::EnemyDangerUpdate() {
	const double warning_threshold = 4.0;
	const double danger_threshold = 3.0;

	std::set<EnemyData>::iterator it;
	for(it = enemies_.begin(); it != enemies_.end(); ++it) {
		// Get enemy position and velocity
		Eigen::Vector3d pos = it->quad_state.position;
		Eigen::Vector3d vel = it->quad_state.velocity;

		// Vector from quad to team balloon
		Eigen::Vector3d vec_quad2balloon = team_balloon_ - pos;

		// Get distance to balloon and project velocity in distance vector
		const double dist_balloon = vec_quad2balloon.norm();
		const double vel_proj = vec_quad2balloon.normalized().dot(vel);

		// Estimate time for enemy to reach balloon
		double time;
		double p0 = 0.0, pf = dist_balloon, v0 = vel_proj;
		trapezoidal::estimate_final_time(p0, pf, v0, 
									     max_vel_, max_acc_, &time);

		// Set quad danger
		if (time < danger_threshold) {
			it->danger.State = it->danger.DANGER;
			// ROS_INFO("Danger!");
		} else if (time < warning_threshold) {
			it->danger.State = it->danger.WARNING;
			// ROS_INFO("WARNING!");
		} else {
			it->danger.State = it->danger.NEUTRAL;
		}
	}
}

void TeamStrategy::GetDangerousEnemies(std::vector<std::string> *names,
	                                   std::vector<std::set<EnemyData>::iterator> *iterators) {
	std::set<EnemyData>::iterator it;
	for(it = enemies_.begin(); it != enemies_.end(); ++it) {
		if(it->danger.State == it->danger.DANGER) {
			// Only return enemies that are not being targeted by teammates
			if(!it->targeted) {
				names->push_back(it->name);
				iterators->push_back(it);
			}
		}
	}
}

/*
void TeamStrategy::GetDangerousDefenders(std::vector<std::string> *names,
	                                   std::vector<std::set<EnemyData>::iterator> *iterators) {
	std::set<EnemyData>::iterator it;
	//only iterate the defenders
	for(it = enemies_.begin(); it != enemies_.end(); ++it) {
		if(it->danger.State == it->danger.DANGER) {
			// Only return enemies that are not being targeted by teammates
			if(!it->targeted) {
				names->push_back(it->name);
				iterators->push_back(it);
			}
		}
	}
}
*/


// Rules to update defensive and offensive state machine
void TeamStrategy::UpdateAttDefStateMachine() {
	std::set<QuadData>::iterator it;
///--------------------------
	std::set<EnemyData>::iterator itEnemies;
	itEnemies=enemies_.begin();
///--------------------------
	for(it = quads_.begin(); it != quads_.end(); ++it)  {
		// Defensive update
		if((it->role.State == it->role.GOALKEEPER) ||
		   (it->role.State == it->role.DEFENSIVE_RIGHT) ||
		   (it->role.State == it->role.DEFENSIVE_LEFT) ||
		   (it->role.State == it->role.DEFENSIVE_CENTRAL)) {
			this->UpdateDefensive(it);
		} else if((it->role.State == it->role.OFFENSIVE_RIGHT) ||
				  (it->role.State == it->role.OFFENSIVE_LEFT) ||
				  (it->role.State == it->role.OFFENSIVE_CENTRAL)) {
			this->UpdateOffensive(it,itEnemies);
		}
	}
}

void TeamStrategy::UpdateOffensive(const std::set<QuadData>::iterator &it,
									const std::set<EnemyData>::iterator &itEnemies) {

	// This strategy makes a quad go back home
	if(it->role.AttackState.State == it->role.AttackState.RETURNING) {
		// Can't get out of this state
	}

 
	// If advancing, go to pop the balloon if passes through 
	// balloon plane
	if(it->role.AttackState.State == it->role.AttackState.ADVANCING) {
		Eigen::Vector3d pos = it->quad_state.position;

		// If a teammate is already targetting ballon or balloon is already popped
		if(balloon_popped_ || balloon_targeted_) {
			it->role.AttackState.State = it->role.AttackState.RETURNING;
		}
		
		// Vector from quad to enemy balloon
		double dist;
		enemy_balloon_plane_.DistancePoint2Plane(pos, &dist);
		if(dist < 0) {  // dist < 0 imples quad beyond balloon
			it->role.AttackState.State = it->role.AttackState.BALLOON;
			balloon_targeted_ = true;
		}
	}

	// If pops the balloon, go to returning mode
	if(it->role.AttackState.State == it->role.AttackState.BALLOON) {
		Eigen::Vector3d pos = it->quad_state.position;
		if((enemy_balloon_ - pos).norm() < 0.1) {  // Balloon probably popped
			it->role.AttackState.State = it->role.AttackState.RETURNING;
			balloon_popped_ = true;
		}
	}
	
	/*
	// If it gets stucked against a defender
	if(it->role.AttackState.State == it->role.AttackState.ADVANCING) {

		//FInding our attacker state
		Eigen::Vector3d pos ;
		Eigen::Vector3d vel ;

		std::set<QuadData>::iterator it_attacker;
		std::set<QuadData>::iterator it2;


		for(it2 = quads_.begin(); it2 != quads_.end(); ++it2)  {
			if(it2->role.State == it->role.OFFENSIVE_LEFT){
				it_attacker=it2;
			}
		}

		pos=it_attacker->quad_state.position;
		vel = it_attacker->quad_state.velocity;
		//std::cout<<it_attacker->name<<std::endl;
		//std::cout<<"our attacker's position"<<std::endl;
		//std::cout<<pos<<std::endl;
	
		
		//Finding enemy defender state

		Eigen::Vector3d posDef;
		Eigen::Vector3d velDef ;

		std::set<EnemyData>::iterator it_defender;
		std::set<EnemyData>::iterator itEnemies2;
	

	for(itEnemies2 = enemies_.begin(); itEnemies2 != enemies_.end(); ++itEnemies2)  {
		posDef=itEnemies2->quad_state.position;
		velDef=itEnemies2->quad_state.velocity;

		Eigen::Vector3d distanceAD=pos-posDef;


		if((distanceAD.norm()<0.8)&&(velDef.norm()<0.5)){
			//std::cout<<"our attacker is returning home"<<std::endl;
			it->role.AttackState.State = it->role.AttackState.BACK;

			}
	}

		//std::cout<<"Enemy defender position"<<std::endl;
		//std::cout<<posDef<<std::endl;
	}

		// If it gets stucked against a defender
	//if(it->role.AttackState.State == it->role.AttackState.BACK) {



	//}
*/

}


void TeamStrategy::UpdateDefensive(const std::set<QuadData>::iterator &it) {
	std::vector<std::string> dangerous_quads;
	std::vector<std::set<EnemyData>::iterator> dangerous_iterator;
	this->GetDangerousEnemies(&dangerous_quads, &dangerous_iterator);

// Keep quad steady at its initial position until a dangerous
// enemy is seen
if(it->role.DefenseState.State == it->role.DefenseState.STEADY) {
if(dangerous_quads.size() > 0) {
it->role.DefenseState.State = it->role.DefenseState.TARGETING;
it->role.DefenseState.target_name = dangerous_quads[0];
dangerous_iterator[0]->targeted = true;
}
}

// If targeting an enemy, track him until enemy is no longer dangerous
if(it->role.DefenseState.State == it->role.DefenseState.TARGETING) {
std::set<EnemyData>::iterator it_target;
FindEnemyIndex(it->role.DefenseState.target_name, &it_target);
if (it_target->danger.State == it_target->danger.NEUTRAL) {
it->role.DefenseState.State = it->role.DefenseState.RETURNING;
it->role.DefenseState.target_name = "";
it_target->targeted = false;
}
}


// If ended targeting an enemy, return to initial position
if(it->role.DefenseState.State == it->role.DefenseState.RETURNING) {
Eigen::Vector3d pos = it->quad_state.position;
if((it->init_pos - pos).norm() < 0.5) { // Close to initial position
it->role.DefenseState.State = it->role.DefenseState.STEADY;
}
}

	
}

// Strategy-dependent methods -----------------------------------
void TeamStrategy::UpdateReferences(const double &dt) {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it)  {
		// Defensive update
		if((it->role.State == it->role.GOALKEEPER) ||
		   (it->role.State == it->role.DEFENSIVE_RIGHT) ||
		   (it->role.State == it->role.DEFENSIVE_LEFT) ||
		   (it->role.State == it->role.DEFENSIVE_CENTRAL)) {
			if(it->role.DefenseState.State == it->role.DefenseState.STEADY) {
				this->DefensiveSteady(it, dt);
			} else if(it->role.DefenseState.State == it->role.DefenseState.TARGETING) {
				this->DefensiveTargeting(it, dt);
			} else if(it->role.DefenseState.State == it->role.DefenseState.RETURNING) {
				this->DefensiveReturn(it, dt);
			}
			///--------------
			else if(it->role.DefenseState.State == it->role.DefenseState.FORWARD) {
				this->DefensiveForward(it, dt);
			}
			//-----------------
		
		} else if((it->role.State == it->role.OFFENSIVE_RIGHT) ||
				  (it->role.State == it->role.OFFENSIVE_LEFT) ||
				  (it->role.State == it->role.OFFENSIVE_CENTRAL)) {
			if(it->role.AttackState.State == it->role.AttackState.RETURNING) {
				this->OffensiveReturn(it, dt);
			} else if(it->role.AttackState.State == it->role.AttackState.ADVANCING) {
				this->OffensiveAdvance(it, dt);
			} else if(it->role.AttackState.State == it->role.AttackState.BALLOON) {
				this->OffensiveBalloon(it, dt);
			}

		}
	}
}

mg_msgs::PVA TeamStrategy::GetRefRk4(const std::set<QuadData>::iterator &it,
			                 		 const double &dt) {

	// Get outputs from the reference_integrator
	geometry_msgs::Point pos_ref;
	geometry_msgs::Vector3 pos_dot_ref, pos_ddot_ref;
	it->reference_integrator.GetPos(&pos_ref);
	it->reference_integrator.GetVel(&pos_dot_ref);
	it->reference_integrator.GetAcc(&pos_ddot_ref);

	// Populate structure for new reference data
	mg_msgs::PVA reference;
	
	reference.Pos = pos_ref;
	reference.Vel = pos_dot_ref;
	reference.Acc = pos_ddot_ref;
	reference.yaw = it->reference.yaw;

	// Set new reference data
	return reference;
}

void TeamStrategy::OffensiveReturn(const std::set<QuadData>::iterator &it,
	                               const double &dt) {
	double kd = 2.0;

	// Get current position/velocity of vehicle
	Eigen::Vector3d pos = it->quad_state.position;
	Eigen::Vector3d vel = it->quad_state.velocity;
    
    // Vector from quad to its initial position
	Eigen::Vector3d vec_quad2init_pos = (it->init_pos - pos);
	
	// References leading to initial position
	if (vec_quad2init_pos.norm() > max_acc_/kd) {
		Eigen::Vector3d ref_vel = max_vel_*vec_quad2init_pos.normalized();
		Eigen::Vector3d ref_acc = kd*(ref_vel - vel);
		it->reference_integrator.SetPos(pos);
		it->reference_integrator.UpdateStates(ref_acc, dt);
	} else {
		Eigen::Vector3d ref_pos = it->init_pos;
		it->reference_integrator.ResetStates(ref_pos);
	}

	it->reference = this->GetRefRk4(it, dt);
}

void TeamStrategy::OffensiveAdvance(const std::set<QuadData>::iterator &it,
	                                const double &dt) {
	double kd = 2.0;



	// Find nearest point in balloon plane
	Eigen::Vector3d pos = it->quad_state.position;
	Eigen::Vector3d vel = it->quad_state.velocity;
    
    Eigen::Vector3d nearest_point;		

	// Vector from quad to enemy balloon plane
	Eigen::Vector3d vec_quad2balloon =(enemy_balloon_-pos).normalized();
	
	// Force leading towards enemy balloon plane
	Eigen::Vector3d ref_vel = max_vel_*vec_quad2balloon;
	Eigen::Vector3d ref_acc = kd*(ref_vel - vel);

	// Set position reference as current, update rk4 based on
	// velocity error
	it->reference_integrator.SetPos(pos);
	it->reference_integrator.UpdateStates(ref_acc, dt);

	it->reference = this->GetRefRk4(it, dt);
}

void TeamStrategy::OffensiveBalloon(const std::set<QuadData>::iterator &it,
	                                const double &dt) {
	double kd = 2.0, kp = 3.0;

	// Get position
	Eigen::Vector3d pos = it->quad_state.position;
	
	// Reference position: enemy balloon
	Eigen::Vector3d ref_pos = enemy_balloon_;

	// Set rk4 reference to balloon
	it->reference_integrator.ResetStates(ref_pos);

	it->reference = this->GetRefRk4(it, dt);
}

void TeamStrategy::DefensiveForward(const std::set<QuadData>::iterator &it,
                         const double &dt){

	double kd = 2.0, kp = 3.0;


	// Get current position/velocity of vehicle
	Eigen::Vector3d pos = it->quad_state.position;
	Eigen::Vector3d vel = it->quad_state.velocity;
    
	Eigen::Vector3d v1 ;
	v1<<2,0,0;

	Eigen::Vector3d vec_quad2back_pos ;
	vec_quad2back_pos=pos+v1;

	
	// Force leading towards initial position
	if (v1.norm() > max_acc_/kp) {
	Eigen::Vector3d ref_vel = max_vel_*v1.normalized();
	Eigen::Vector3d ref_acc = kd*(ref_vel - vel);
	it->reference_integrator.SetPos(pos);
	it->reference_integrator.UpdateStates(ref_acc, dt);
	} else {
		Eigen::Vector3d ref_pos =vec_quad2back_pos ;
		it->reference_integrator.ResetStates(ref_pos);
	}



	it->reference = this->GetRefRk4(it, dt);



    }

void TeamStrategy::DefensiveSteady(const std::set<QuadData>::iterator &it,
	                               const double &dt) {
	Eigen::Vector3d ref_pos = it->init_pos;
	it->reference_integrator.ResetStates(ref_pos);

	it->reference = this->GetRefRk4(it, dt);
	///////////////

}

void TeamStrategy::DefensiveTargeting(const std::set<QuadData>::iterator &it,
	                                   const double &dt) {
	double kd = 5.0, kp1 = 2.0;

	// Get position and velocity
	Eigen::Vector3d pos = it->quad_state.position;
	Eigen::Vector3d vel = it->quad_state.velocity;

	// Find position of enemy
	std::set<EnemyData>::iterator it_target;
	FindEnemyIndex(it->role.DefenseState.target_name, &it_target);
	Eigen::Vector3d pos_target = it_target->quad_state.position;
	Eigen::Vector3d vel_target = it_target->quad_state.velocity;

	// Define plane with normal towards enemy base, and origin at defense line
	Eigen::Vector3d init_pos = it->init_pos;
	Plane3d defense_plane(init_pos, offensive_direction_);

	// Project enemy position onto plane, finding position reference
	Eigen::Vector3d pos_projection;  // Enemy projection onto plane
	double dist;					 // Enemy distance to plane
	defense_plane.ProjectPointOntoPlane(pos_target, &pos_projection, &dist);

	// Project enemy velocity onto plane
	Eigen::Vector3d vel_projection;
	defense_plane.ProjectVectorOntoPlane(vel_target, &vel_projection);

	// Reference position: plane "advance_dist" ahead of defense line
	const double advance_dist = 1.0;
	Eigen::Vector3d ref_pos = pos_projection + advance_dist*offensive_direction_;

	// Reference velocity is sum of two terms:
	// 1) enemy projection onto defense plane
	// 2) enemy velocity projected onto plane
	Eigen::Vector3d ref_vel_pos = kp1*(ref_pos - pos);
	Eigen::Vector3d ref_vel = ref_vel_pos + vel_projection;
	Eigen::Vector3d ref_acc = kd*(ref_vel - vel);

	// Set position reference as current, update rk4 based on
	// velocity only
	it->reference_integrator.SetPos(ref_pos);
	it->reference_integrator.UpdateStates(ref_acc, dt);

	it->reference = this->GetRefRk4(it, dt);
}

void TeamStrategy::DefensiveReturn(const std::set<QuadData>::iterator &it,
	                               const double &dt) {
	double kd = 2.0, kp = 3.0;

	// Get current position/velocity of vehicle
	Eigen::Vector3d pos = it->quad_state.position;
	Eigen::Vector3d vel = it->quad_state.velocity;
    
    // Vector from quad to its initial position
	Eigen::Vector3d vec_quad2init_pos = (it->init_pos - pos);
	
	// Force leading towards initial position
	if (vec_quad2init_pos.norm() > max_acc_/kp) {
		Eigen::Vector3d ref_vel = max_vel_*vec_quad2init_pos.normalized();
		Eigen::Vector3d ref_acc = kd*(ref_vel - vel);
		it->reference_integrator.SetPos(pos);
		it->reference_integrator.UpdateStates(ref_acc, dt);
	} else {
		Eigen::Vector3d ref_pos = it->init_pos;
		it->reference_integrator.ResetStates(ref_pos);
	}

	it->reference = this->GetRefRk4(it, dt);
}