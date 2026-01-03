#ifndef PT100_LOGGER_DIAGNOSTICS_MESH_H_
#define PT100_LOGGER_DIAGNOSTICS_MESH_H_

#include <stdbool.h>

#include "diagnostics/diag_common.h"
#include "runtime_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Execute RunDiagMesh.
 * @param runtime Parameter runtime.
 * @param full Parameter full.
 * @param start Parameter start.
 * @param stop Parameter stop.
 * @param force_root Parameter force_root.
 * @param timeout_ms Parameter timeout_ms.
 * @param verbosity Parameter verbosity.
 * @return Return the function result.
 */
int RunDiagMesh(const app_runtime_t* runtime,
                bool full,
                bool start,
                bool stop,
                bool force_root,
                int timeout_ms,
                diag_verbosity_t verbosity);

#ifdef __cplusplus
}
#endif

#endif // PT100_LOGGER_DIAGNOSTICS_MESH_H_
