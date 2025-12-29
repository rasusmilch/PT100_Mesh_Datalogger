#ifndef PT100_LOGGER_RUNTIME_HEALTH_PUBLISHER_H_
#define PT100_LOGGER_RUNTIME_HEALTH_PUBLISHER_H_

#include "runtime_state.h"

#ifdef __cplusplus
extern "C" {
#endif

  void RuntimeHealthPublisherInit(runtime_state_t* state);

  void RuntimeHealthPublisherTick(runtime_state_t* state);

  static inline void RuntimeHealthMarkDirty(runtime_state_t* state)
  {
    if (state == NULL) {
      return;
    }
    state->health_publisher.dirty = true;
  }

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUNTIME_HEALTH_PUBLISHER_H_
