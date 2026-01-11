#ifndef PT100_LOGGER_MEM_POOL_H_
#define PT100_LOGGER_MEM_POOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#ifdef __cplusplus
extern "C" {
#endif

  typedef struct
  {
    uint8_t* buffer;
    size_t block_size;
    size_t block_count;
    size_t free_count;
    void* free_list;
    portMUX_TYPE lock;
  } mem_pool_t;

  size_t MemPoolGetRequiredBytes(size_t block_size, size_t block_count);
  bool MemPoolInit(mem_pool_t* pool,
                   void* buffer,
                   size_t block_size,
                   size_t block_count);
  void* MemPoolAlloc(mem_pool_t* pool);
  void MemPoolFree(mem_pool_t* pool, void* block);

#ifdef __cplusplus
}
#endif

#endif  // PT100_LOGGER_MEM_POOL_H_
