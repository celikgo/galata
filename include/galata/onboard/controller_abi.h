// SPDX-License-Identifier: Apache-2.0
#ifndef GALATA_ONBOARD_CONTROLLER_ABI_H
#define GALATA_ONBOARD_CONTROLLER_ABI_H

/*
 * Stable C boundary for a target controller shared object.
 *
 * The ABI intentionally carries only POD values and caller-owned buffers. A
 * controller must never throw across this boundary, retain frame pointers
 * after a callback returns, or write more than the declared actuator count.
 * The host validates sequence, timestamp, finiteness and channel width again
 * before an actuator frame can reach a guarded transport.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GALATA_CONTROLLER_ABI_VERSION 2u

typedef struct GalataControllerSensorFrame {
  uint64_t sequence;
  double timestamp_s;
  size_t value_count;
  const double* values;
} GalataControllerSensorFrame;

typedef struct GalataControllerActuatorFrame {
  size_t value_count;
  double* values;
} GalataControllerActuatorFrame;

typedef uint32_t (*GalataControllerAbiVersionFn)(void);

/*
 * Return NULL on failure. The manifest and model path bytes are borrowed for
 * this call. The model path points to the verified model artifact selected by
 * the runtime; a controller may load it during creation but must not retain
 * the pointer after this call.
 */
typedef void* (*GalataControllerCreateFn)(const char* manifest,
                                          size_t manifest_size,
                                          const char* model_path,
                                          size_t model_path_size,
                                          size_t sensor_count,
                                          size_t actuator_count);

/* Return zero on success. On failure, write a short diagnostic if possible. */
typedef int (*GalataControllerStepFn)(void* context,
                                      const GalataControllerSensorFrame* sensor,
                                      GalataControllerActuatorFrame* actuator,
                                      char* diagnostic,
                                      size_t diagnostic_size);

typedef void (*GalataControllerDestroyFn)(void* context);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* GALATA_ONBOARD_CONTROLLER_ABI_H */
