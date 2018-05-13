
#include "ml_strategy/callbacks.h"

namespace callbacks {

// Trigger the game start
void GameStartCallback(const std_msgs::Empty& empty_msg) {
	pthread_mutex_lock(&mutexes_.m_team_strategy);
		globals_.obj_team_strategy.SetStartGame();
	pthread_mutex_unlock(&mutexes_.m_team_strategy);
}

// Get state of the game (all quads' position and velocity)
void GameStateCallback(const mg_msgs::GameState::ConstPtr& msg) {
	uint n_quads = msg->GameState.size();
	pthread_mutex_lock(&mutexes_.m_team_strategy);
		for (uint i = 0; i < n_quads; i++) {
			globals_.obj_team_strategy.
				UpdateQuadOdom(msg->GameState[i].child_frame_id, 
					           msg->GameState[i]);
		}
	pthread_mutex_unlock(&mutexes_.m_team_strategy);
}

// Callback to react for land all quads request
void LandAllQuadsCallback(const std_msgs::Empty& empty_msg) {
	pthread_mutex_lock(&mutexes_.m_team_strategy);
		globals_.obj_team_strategy.SetQuadsToLand();
	pthread_mutex_unlock(&mutexes_.m_team_strategy);	
}

}  // namespace callbacks