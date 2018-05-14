
#include "ml_strategy/globals.h"
#include "mg_msgs/SetQuadBool.h"

#include <ros/ros.h>
#include "std_msgs/Empty.h"

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
	


	//Using estimation from data set, assuming we are on the red team
	/*
	Estimated Red Position: 6.15297 -0.789981 2.16978
	Estimated Blue Position: -7.27838 0.714218 1.29838
	*/
	//Eigen::Vector3d enemy_balloon_pos(6.15297,-0.789981 ,2.16978);
	//Eigen::Vector3d team_balloon_pos(-7.27838,0.714218,1.29838);


	// Initialize strategy class --------------------------------
	globals_.obj_team_strategy =
		TeamStrategy(max_vel, max_acc, team_balloon_pos, enemy_balloon_pos);

	//To add and remove shields, setup
	ros::ServiceClient shield_client = 
		node.serviceClient<mg_msgs::SetQuadBool>("/mediation_layer/set_quad_shield");
	mg_msgs::SetQuadBool srv_msg;
	srv_msg.request.set_bool = 1;

	// Set quad roles based on the number of quads --------------
	std::vector<uint> roles;
	QuadRole role_struct;
	if(quad_names.size() == 2) {
		roles = {role_struct.GOALKEEPER, 
			     role_struct.OFFENSIVE_CENTRAL};
	} else if(quad_names.size() == 3) {
		roles = {role_struct.GOALKEEPER,
					role_struct.PUSHER,
				 //role_struct.OFFENSIVE_RIGHT,
			     //role_struct.OFFENSIVE_LEFT,
				role_struct.OFFENSIVE_RIGHT	};
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
				
			//Add shields depending on Quad's role
			if(roles[i]==role_struct.GOALKEEPER){
				srv_msg.request.quad_name = quad_names[i];
				shield_client.call(srv_msg);
			}

			if(roles[i]==role_struct.PUSHER){
				srv_msg.request.quad_name = quad_names[i];
				shield_client.call(srv_msg);
			}
		

			//Remove this
			//fOR BLOONS , add shield to harry
			//srv_msg.request.quad_name = "harry";
			//shield_client.call(srv_msg);


		}
	}

	// Callbacks ---------------------------------------------------
  	ros::Subscriber game_state_sub = node.subscribe<mg_msgs::GameState>
  				("/mediation_layer/Game_State", 10, callbacks::GameStateCallback);
	ros::Subscriber start_game_sub = node.subscribe
				("/mediation_layer/Start_Game", 10, callbacks::GameStartCallback);
	ros::Subscriber land_quads_sub = node.subscribe
				("/mediation_layer/land_quads", 10, callbacks::LandAllQuadsCallback);
	
	//Hardcoding enemy ready
	//Setup  Signal ready for game!
				/*
				
	ros::ServiceClient ready_client=
		node.serviceClient<mg_msgs::SetQuadBool>("/mediation_layer/set_quad_ready");
	mg_msgs::SetQuadBool srv_msg_ready;
	srv_msg_ready.request.set_bool=1;

	srv_msg_ready.request.quad_name="hermione";
	ready_client.call(srv_msg_ready);
	srv_msg_ready.request.quad_name="ron";
	ready_client.call(srv_msg_ready);
	srv_msg_ready.request.quad_name="harry";
	ready_client.call(srv_msg_ready);
		srv_msg_ready.request.quad_name="gryphon";
	ready_client.call(srv_msg_ready);
	srv_msg_ready.request.quad_name="phoenix";
	ready_client.call(srv_msg_ready);
	srv_msg_ready.request.quad_name="pegasus";
	ready_client.call(srv_msg_ready);
	*/

	//--------------------------------------------------------------
/*
	//Setup  Signal ready for game!
	ros::ServiceClient ready_client=
		node.serviceClient<mg_msgs::SetQuadBool>("/mediation_layer/set_quad_ready");
	mg_msgs::SetQuadBool srv_msg_ready;

	globals_.obj_team_strategy.quad_names_=quad_names;

	//Checking if we are ready to play(pos ==init_pos  ) and send service message
	//int ready=globals_.obj_team_strategy.CheckReady(quad_names,globals_.obj_team_strategy.init_pos_game);

	if(globals_.obj_team_strategy.ready2play_==true){
		//im ready

		for (uint i = 0; i < quad_names.size(); i++) {
			srv_msg_ready.request.quad_name=quad_names[i];
			ready_client.call(srv_msg_ready);
		}

	}

	//srv_msg_ready.request.set_bool=globals_.obj_team_strategy.ready2play_;//asdaaaaa
	srv_msg_ready.request.set_bool=1;
	*/
	//----------------------------------------------------------------------
				

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