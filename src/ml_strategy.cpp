
#include "ml_strategy/globals.h"




// roscore
// roslaunch mediation_layer mediationLayer_withDynamics.launch
// roslaunch ml_strategy_students play_against_marcelino.launch
// cd ~/catkin_ws && catkin_make



// source ~/catkin_ws/install/setup.bash && cd ~/Tournament_Files && roslaunch mediationLayer_withDynamics.launch
// source ~/catkin_ws/install/setup.bash && cd ~/Tournament_Files && roslaunch 09_overdogs_vs_underdogs.launch
// cd ~/catkin_ws && catkin_make install && source ~/catkin_ws/install/setup.bash && cd ~/Tournament_Files && roslaunch 09_overdogs_vs_underdogs.launch


// Global variables--------------------------
globalVariables globals_;
mutexClass mutexes_;

void GameStateCallback(const mg_msgs::GameState::ConstPtr& msg) {
	uint n_quads = msg->GameState.size();
	pthread_mutex_lock(&mutexes_.m_team_strategy);
	for (uint i = 0; i < n_quads; i++) {
		globals_.obj_team_strategy.
			UpdateQuadOdom(msg->GameState[i].child_frame_id, 
				           msg->GameState[i]);
	}
	pthread_mutex_unlock(&mutexes_.m_team_strategy);
	// UpdateQuadOdom(const std::string &name, 
 //                                  const nav_msgs::Odometry &odom)
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ml_strategy");
	ros::NodeHandle node("~");
  	ROS_INFO("Marcelino's strategy started!");

  	// Get team quads
	std::vector<std::string> quad_names;
	std::vector<double> init_pos, init_yaw;
	node.getParam("MyTeam", quad_names);
	node.getParam("InitialPosition", init_pos);
	node.getParam("InitialYaw", init_yaw);

	// Get parameters of dynamics -------------------------------
	double max_acc, max_vel;
	node.getParam("max_acceleration", max_acc);
	node.getParam("max_velocity", max_vel);

	// Get balloon positions
	std::vector<double> team_balloon, enemy_balloon;
	node.getParam("TeamBalloon", team_balloon);
	node.getParam("EnemyBalloon", enemy_balloon);
	Eigen::Vector3d team_balloon_pos(team_balloon[0], team_balloon[1], team_balloon[2]);
	Eigen::Vector3d enemy_balloon_pos(enemy_balloon[0], enemy_balloon[1], enemy_balloon[2]);

	// Initialize strategy class --------------------------------
	globals_.obj_team_strategy =
		TeamStrategy(max_vel, max_acc, team_balloon_pos, enemy_balloon_pos);

	// Set quad roles based on the number of quads --------------
	std::vector<uint> roles;
	QuadRole role_struct;
	if(quad_names.size() == 2) {
		roles = {role_struct.GOALKEEPER, 
			     role_struct.OFFENSIVE_CENTRAL};
	} else if(quad_names.size() == 3) {
		roles = {role_struct.GOALKEEPER,
					role_struct.GOALKEEPER,
				 //role_struct.OFFENSIVE_RIGHT,
			     //role_struct.OFFENSIVE_LEFT,
				role_struct.OFFENSIVE_LEFT	};
	} else {
		roles.resize(quad_names.size());
	}

	if (float(quad_names.size()) > float(init_pos.size())/3.0) {
		ROS_ERROR("[ml_strategy]: Initial positions not well defined!");
		return 0;
	} else 	if (float(quad_names.size()) > float(init_yaw.size())) {
		ROS_ERROR("[ml_strategy]: Initial yaw angles not well defined!");
		return 0;
	} else {
		// Add team quads
		for (uint i = 0; i < quad_names.size(); i++) {
			Eigen::Vector3d pos(init_pos[3*i], init_pos[3*i+1], init_pos[3*i+2]);
			std::string output_topic = "/" + quad_names[i] + "/px4_control/PVA_Ref";
			ROS_INFO("[ml_strategy]: Created publisher: %s", output_topic.c_str());
			globals_.obj_team_strategy.
				AddQuad(quad_names[i], roles[i], pos, init_yaw[i],
				        output_topic, &node);
		}
	}

  	ros::Subscriber game_state_sub = node.subscribe<mg_msgs::GameState>
  				("/mediation_layer/Game_State", 10, GameStateCallback);
  
    // Threads -------------------------------------------
  	std::thread h_strategy_thread;
  	const double strategy_rate = 30;
  	h_strategy_thread = std::thread(threads::ML_StrategyThread, strategy_rate);


	// ROS loop that starts callbacks/publishers
	ros::spin();

	// Kill mutexes
	mutexes_.destroy();

	return 0;

}