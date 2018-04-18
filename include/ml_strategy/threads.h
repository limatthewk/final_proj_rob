#ifndef THREADS_H_
#define THREADS_H_

#include "ml_strategy/globals.h"

namespace threads {

// Thread that runs the strategy
void ML_StrategyThread(const double &rate);

}  // namespace threads

#endif  // THREADS_H_