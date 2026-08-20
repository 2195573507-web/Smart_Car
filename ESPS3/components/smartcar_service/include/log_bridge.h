#ifndef SMARTCAR_LOG_BRIDGE_H
#define SMARTCAR_LOG_BRIDGE_H

#include "scbp_parser.h"

void log_bridge_handle(const scbp_can_frame_t *frame);

#endif /* SMARTCAR_LOG_BRIDGE_H */
