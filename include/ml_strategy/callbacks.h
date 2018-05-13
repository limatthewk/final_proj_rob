#ifndef CALLBACKS_H_
#define CALLBACKS_H_

#include "ml_strategy/globals.h"

namespace callbacks {

// Trigger the game start
void GameStartCallback(const std_msgs::Empty& empty_msg);

// Get state of the game (all quads' position and velocity)
void GameStateCallback(const mg_msgs::GameState::ConstPtr& msg);

// Callback to react for land all quads request
void LandAllQuadsCallback(const std_msgs::Empty& empty_msg);

}  // namespace callbacks

#endif  // CALLBACKS_H_