#ifndef PT100_LOGGER_MESH_ADDR_H_
#define PT100_LOGGER_MESH_ADDR_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
  uint8_t addr[6];
} pt100_mesh_addr_t;

static inline pt100_mesh_addr_t
Pt100MeshAddrFromMac(const uint8_t mac[6])
{
  pt100_mesh_addr_t addr;
  memcpy(addr.addr, mac, sizeof(addr.addr));
  return addr;
}

static inline bool
Pt100MeshAddrIsZero(const pt100_mesh_addr_t* a)
{
  static const pt100_mesh_addr_t kZeroAddr = { 0 };
  return (a == NULL) || (memcmp(a, &kZeroAddr, sizeof(*a)) == 0);
}

#endif // PT100_LOGGER_MESH_ADDR_H_
