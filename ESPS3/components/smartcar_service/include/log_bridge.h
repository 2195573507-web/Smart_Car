#ifndef SMARTCAR_LOG_BRIDGE_H
#define SMARTCAR_LOG_BRIDGE_H

#include "srp_codec.h"
#include "srp_registry.h"

void log_bridge_handle(const srp_frame_t *frame);

#endif /* SMARTCAR_LOG_BRIDGE_H */
