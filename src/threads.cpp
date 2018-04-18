#include "ml_strategy/threads.h"

namespace threads {

void ML_StrategyThread(const double &rate) {
    ROS_DEBUG("[ml_strategy]: ML_Strategy Thread started!");

    // Rate at which this thread will run
    double dt = 1/rate;
    ros::Rate loop_rate(rate);

    // Send first reference position
    pthread_mutex_lock(&mutexes_.m_team_strategy);
        globals_.obj_team_strategy.PublishReferences();
    pthread_mutex_unlock(&mutexes_.m_team_strategy);

    // Run the strategy loop
    while (ros::ok()) {
	    pthread_mutex_lock(&mutexes_.m_team_strategy);

	    	// Update danger of enemies
	    	globals_.obj_team_strategy.EnemyDangerUpdate();

	    	// Update offensive / defensive tactics
	    	globals_.obj_team_strategy.UpdateAttDefStateMachine();

	    	// Update PVA references
	    	globals_.obj_team_strategy.UpdateReferences(dt);

	    	// Publish references
	        globals_.obj_team_strategy.PublishReferences();
	    pthread_mutex_unlock(&mutexes_.m_team_strategy);

    	loop_rate.sleep();
    }
    ROS_DEBUG("[ml_strategy]: Exiting ML_Strategy Thread...");
}

}  // namespace threads