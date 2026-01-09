#ifndef PT100_LOGGER_RUN_GPIO_H_
#define PT100_LOGGER_RUN_GPIO_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Execute RunGpioInit.
   */
  void RunGpioInit(void);

  /**
   * @brief Execute RunGpioStopActive.
   * @return Return the function result.
   */
  bool RunGpioStopActive(void);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_RUN_GPIO_H_
