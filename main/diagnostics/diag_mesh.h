#ifndef PT100_LOGGER_DIAGNOSTICS_MESH_H_
#define PT100_LOGGER_DIAGNOSTICS_MESH_H_

#include <stdbool.h>

#include "diagnostics/diag_common.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

int RunDiagMesh(const app_runtime_t* runtime, bool full, bool start, bool stop, bool verbose);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_MESH_H_
