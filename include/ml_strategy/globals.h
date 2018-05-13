#include <ros/ros.h>
#include <thread>
#include <Eigen/Dense>

// Defined libraries
#include "ml_strategy/threads.h"
#include "mg_msgs/GameState.h"
#include "ml_strategy/callbacks.h"


// Structs for global variables
#include "ml_strategy/structs.h"

// Service type (for toggling shield)
#include "mg_msgs/SetQuadBool.h"

#include "std_msgs/Empty.h"

// Declare global variables
extern globalVariables globals_;
extern mutexClass mutexes_;