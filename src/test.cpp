void TeamStrategy::OffensiveAdvance(const std::set<QuadData>::iterator &it,
	                                const double &dt) {
	double kd = 2.0;
	// Find nearest point in balloon plane
	Eigen::Vector3d pos = it->quad_state.position;
	Eigen::Vector3d vel = it->quad_state.velocity;
    
    Eigen::Vector3d nearest_point;		

	// Vector from quad to enemy balloon plane
	//Eigen::Vector3d vec;

	//vec << 0,0,0;
	// 0,0,- is up
	// 0,0,+ is down
	// 0,+,0 is right
	// 0,-,0 is left
	// -,0,0 is back
	// +,0,0 is forward


	Eigen::Vector3d vec_quad2balloon =(enemy_balloon_ - pos).normalized();
	Eigen::Vector3d dist_quad2balloon =enemy_balloon_ - pos;
	//std::cout << vec_quad2balloon << std::endl;
	//std::cout << std::abs(dist_quad2balloon[0]) << std::endl;

	Eigen::Vector3d vec;
	vec << 0,0,0;

	std::set<EnemyData>::iterator it2;
	for(it2 = enemies_.begin(); it2 != enemies_.end(); ++it2) {
		Eigen::Vector3d e_pos = it2->quad_state.position;
		Eigen::Vector3d e_vel = it2->quad_state.velocity;

		// Vector from quad to team balloon
		Eigen::Vector3d vec_quad2enemy = pos - e_pos;
		Eigen::Vector3d test = vec_quad2enemy.normalized()-vec_quad2balloon;
		//std::cout << vec_quad2enemy << std::endl<<std::endl;
		
		float dist_quad2enemy = sqrt(pow(vec_quad2enemy[0],2.0)+pow(vec_quad2enemy[1],2.0)+pow(vec_quad2enemy[2],2.0));
		float e_vel_mag = sqrt(pow(e_vel[0],2.0)+pow(e_vel[1],2.0)+pow(e_vel[2],2.0));
		float vel_mag = sqrt(pow(vel[0],2.0)+pow(vel[1],2.0)+pow(vel[2],2.0));

		//if (e_vel_mag < 1.5 && vel_mag > 1.5 && dist_quad2enemy < 6){
		if (e_vel_mag < 1.5 && dist_quad2enemy < 6){
			if(it->role.State == it->role.OFFENSIVE_LEFT){
				vec << 0,15,0;
			}
			else if(it->role.State == it->role.OFFENSIVE_RIGHT){
				vec << 0,-15,0;
			}
				std::cout << "enemy is blocking" << std::endl<<std::endl;
				std::cout << "enemy vel mag " << e_vel_mag << std::endl << std::endl;
				std::cout << "enemy velocity" << std::endl << e_vel << std::endl << std::endl;
				std::cout << "quad velocity" << std::endl << vel << std::endl << std::endl;	
				if ((e_vel_mag < .2) && (fabs(vel[0]) < 1.8)){

					it->role.AttackState.State = it->role.AttackState.BALLOON;
					balloon_targeted_ = true;
				}
		}
	}
	// Force leading towards enemy balloon plane
	Eigen::Vector3d ref_vel = max_vel_*((enemy_balloon_ - pos).normalized());
	Eigen::Vector3d ref_acc = kd*(ref_vel - vel)+vec;

	// Set position reference as current, update rk4 based on
	// velocity error
	it->reference_integrator.SetPos(pos);
	//std::cout << ref_acc+vec << std::endl << std::endl;
	it->reference_integrator.UpdateStates(ref_acc, dt);

	it->reference = this->GetRefRk4(it, dt);
}

/*
void TeamStrategy::OffensiveAdvance(const std::set<QuadData>::iterator &it,
	                                const double &dt) {
	double kd = 2.0;



	// Find nearest point in balloon plane
	Eigen::Vector3d pos = it->quad_state.position;
	Eigen::Vector3d vel = it->quad_state.velocity;
    
    Eigen::Vector3d nearest_point;		

	// Vector from quad to enemy balloon plane
	//Eigen::Vector3d vec;

	//vec << 0,0,0;
	// 0,0,- is up
	// 0,0,+ is down
	// 0,+,0 is right
	// 0,-,0 is left
	// -,0,0 is back
	// +,0,0 is forward


	Eigen::Vector3d vec_quad2balloon =(enemy_balloon_ - pos).normalized();
	Eigen::Vector3d dist_quad2balloon =enemy_balloon_ - pos;
	//std::cout << vec_quad2balloon << std::endl;
	//std::cout << std::abs(dist_quad2balloon[0]) << std::endl;

	Eigen::Vector3d vec;
	vec << 0,0,0;

	std::set<EnemyData>::iterator it2;
	for(it2 = enemies_.begin(); it2 != enemies_.end(); ++it2) {
		Eigen::Vector3d e_pos = it2->quad_state.position;
		Eigen::Vector3d e_vel = it2->quad_state.velocity;

		// Vector from quad to team balloon
		Eigen::Vector3d vec_quad2enemy = pos - e_pos;
		Eigen::Vector3d test = vec_quad2enemy.normalized()-vec_quad2balloon;
		//std::cout << vec_quad2enemy << std::endl<<std::endl;
		
		float dist_quad2enemy = sqrt(pow(vec_quad2enemy[0],2.0)+pow(vec_quad2enemy[1],2.0)+pow(vec_quad2enemy[2],2.0));
		float e_vel_mag = sqrt(pow(e_vel[0],2.0)+pow(e_vel[1],2.0)+pow(e_vel[2],2.0));
		//float vel_mag = sqrt(pow(vel[0],2.0)+pow(vel[1],2.0)+pow(vel[2],2.0));

		Eigen::Vector3d pos_dif = pos - e_pos;
		float pos_dif_mag = sqrt(pow(pos_dif[0],2.0)+pow(pos_dif[1],2.0)+pow(pos_dif[2],2.0));


		if (e_vel_mag < 1.5 && dist_quad2enemy < 6){
			if(it->role.State == it->role.OFFENSIVE_LEFT){
				vec << 0,15,0;
			}
			else if(it->role.State == it->role.OFFENSIVE_RIGHT){
				vec << 0,-15,0;
			}
				std::cout << "enemy is blocking" << std::endl<<std::endl;
				std::cout << "enemy vel mag " << e_vel_mag << std::endl << std::endl;
				std::cout << "enemy velocity" << std::endl << e_vel << std::endl << std::endl;
				std::cout << "quad velocity" << std::endl << vel << std::endl << std::endl;	
				if ((e_vel_mag < .2) && (fabs(vel[0]) < 1.8)){

					it->role.AttackState.State = it->role.AttackState.BALLOON;
					balloon_targeted_ = true;
				}
		}



		if (e_vel_mag < 1 && dist_quad2enemy < 5.5){
				//if (e_vel[1] < 0){
				if (it->role.State == it->role.OFFENSIVE_LEFT) {
					vec << 0,15,0;
			
				
				}
				//else if (e_vel[1] > 0){
				else if (it->role.State == it->role.OFFENSIVE_RIGHT){
					vec << 0,-15,0;
				
				}
					std::cout << "enemy is blocking" << std::endl<<std::endl;
					std::cout << "enemy vel mag " << e_vel_mag << std::endl << std::endl;
					std::cout << "enemy velocity" << std::endl << e_vel << std::endl << std::endl;
					std::cout << "quad velocity" << std::endl << vel << std::endl << std::endl;
					//std::cout << "pos dif" << std::endl << pos_dif << std::endl << std::endl;
					//std::cout << "vel dif" << std::endl << vel-e_vel << std::endl << std::endl;
					std::cout << std::endl << pos_dif[1]/pos_dif_mag << std::endl << std::endl;
					std::cout << std::endl << e_pos[1]-pos[1] << std::endl << std::endl;




					
					//if ((e_vel_mag < .2 && (fabs(vel[0]) < 1.8)){
					if ((e_vel_mag < .2 && fabs(pos_dif[1]/pos_dif_mag)>0.4) || (pos_dif[0] < 0 && vec_quad2balloon[0]<0) || (pos_dif[0] > 0 && vec_quad2balloon[0]>0)) {

						it->role.AttackState.State = it->role.AttackState.BALLOON;
						balloon_targeted_ = true;
					}
		}
		
	}

	


		
		
	

	
	// Force leading towards enemy balloon plane
	Eigen::Vector3d ref_vel = max_vel_*((enemy_balloon_ - pos).normalized());
	Eigen::Vector3d ref_acc = kd*(ref_vel - vel)+vec;

	// Set position reference as current, update rk4 based on
	// velocity error
	it->reference_integrator.SetPos(pos);
	//std::cout << ref_acc+vec << std::endl << std::endl;
	it->reference_integrator.UpdateStates(ref_acc, dt);

	it->reference = this->GetRefRk4(it, dt);
}