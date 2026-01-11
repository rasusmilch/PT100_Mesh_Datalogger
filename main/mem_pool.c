#include "mem_pool.h"

#include <string.h>

static size_t
AlignSize(size_t size, size_t alignment)
{
  if (alignment == 0) {
    return size;
  }
  const size_t remainder = size % alignment;
  if (remainder == 0) {
    return size;
  }
  return size + (alignment - remainder);
}

size_t
MemPoolGetRequiredBytes(size_t block_size, size_t block_count)
{
  const size_t aligned_size = AlignSize(block_size, sizeof(void*));
  return aligned_size * block_count;
}

bool
MemPoolInit(mem_pool_t* pool,
            void* buffer,
            size_t block_size,
            size_t block_count)
{
  if (pool == NULL || buffer == NULL || block_size == 0 || block_count == 0) {
    return false;
  }

  const size_t aligned_size = AlignSize(block_size, sizeof(void*));
  memset(pool, 0, sizeof(*pool));
  pool->buffer = (uint8_t*)buffer;
  pool->block_size = aligned_size;
  pool->block_count = block_count;
  pool->free_count = block_count;
  pool->free_list = NULL;
  pool->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

  for (size_t i = 0; i < block_count; ++i) {
    uint8_t* block = pool->buffer + (i * aligned_size);
    *(void**)block = pool->free_list;
    pool->free_list = block;
  }
  return true;
}

void*
MemPoolAlloc(mem_pool_t* pool)
{
  if (pool == NULL) {
    return NULL;
  }

  void* block = NULL;
  portENTER_CRITICAL(&pool->lock);
  if (pool->free_list != NULL) {
    block = pool->free_list;
    pool->free_list = *(void**)pool->free_list;
    pool->free_count -= 1;
  }
  portEXIT_CRITICAL(&pool->lock);
  return block;
}

void
MemPoolFree(mem_pool_t* pool, void* block)
{
  if (pool == NULL || block == NULL) {
    return;
  }

  portENTER_CRITICAL(&pool->lock);
  *(void**)block = pool->free_list;
  pool->free_list = block;
  pool->free_count += 1;
  portEXIT_CRITICAL(&pool->lock);
}
