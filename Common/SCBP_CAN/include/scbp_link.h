#ifndef SCBP_LINK_H
#define SCBP_LINK_H

#include <stdint.h>

#include "scbp_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SCBP_LINK_PENDING_SLOTS UINT8_C(4)
#define SCBP_LINK_ACK_TIMEOUT_MS UINT32_C(500)
#define SCBP_LINK_MAX_RETRIES UINT8_C(3)

typedef enum {
    SCBP_LINK_ACTIVE = 0,
    SCBP_LINK_WARNING,
    SCBP_LINK_PASSIVE,
    SCBP_LINK_BUS_OFF
} scbp_link_state_t;

typedef enum {
    SCBP_LINK_TX_OK = 0,
    SCBP_LINK_TX_TRANSPORT_FAILURE,
    SCBP_LINK_TX_TIMEOUT,
    SCBP_LINK_TX_REMOTE_ERROR,
    SCBP_LINK_TX_BUS_OFF
} scbp_link_tx_result_t;

typedef int (*scbp_link_transport_send_t)(const uint8_t *data, uint16_t length,
                                          void *context);
typedef void (*scbp_link_tx_callback_t)(scbp_link_tx_result_t result,
                                        uint8_t status_code, void *context);
typedef void (*scbp_link_frame_callback_t)(const scbp_can_frame_t *frame,
                                           void *context);
typedef void (*scbp_link_bus_off_callback_t)(void *context);

typedef struct {
    uint8_t local_node;
    uint32_t ack_timeout_ms;
    uint8_t max_retries;
    scbp_link_transport_send_t transport_send;
    scbp_link_frame_callback_t on_frame;
    scbp_link_bus_off_callback_t on_bus_off;
    void *context;
} scbp_link_config_t;

typedef struct {
    uint8_t in_use;
    uint8_t retry_count;
    uint8_t sequence;
    uint16_t can_id;
    uint16_t frame_length;
    uint32_t last_tx_ms;
    scbp_link_tx_callback_t callback;
    void *callback_context;
    uint8_t frame_bytes[SCBP_CAN_MAX_FRAME_SIZE];
} scbp_link_pending_t;

typedef struct {
    uint16_t tec;
    uint16_t rec;
    uint8_t next_sequence;
    scbp_link_state_t state;
    scbp_link_config_t config;
    scbp_link_pending_t pending[SCBP_LINK_PENDING_SLOTS];
} scbp_link_t;

void scbp_link_init(scbp_link_t *link, const scbp_link_config_t *config);
int scbp_link_send(scbp_link_t *link, uint8_t priority, uint8_t destination,
                   uint16_t message_id, uint8_t flags, const uint8_t *payload,
                   uint8_t length, uint32_t now_ms,
                   scbp_link_tx_callback_t callback, void *callback_context);
int scbp_link_send_fast_response(scbp_link_t *link, uint8_t priority,
                                 uint8_t destination, uint8_t is_error,
                                 uint16_t ack_can_id, uint8_t ack_sequence,
                                 uint8_t status_code, uint32_t now_ms);
void scbp_link_receive(scbp_link_t *link, const scbp_can_frame_t *frame);
void scbp_link_report_parser_error(scbp_link_t *link, scbp_parser_error_t error);
void scbp_link_tick(scbp_link_t *link, uint32_t now_ms);
void scbp_link_recover(scbp_link_t *link);
uint16_t scbp_link_get_tec(const scbp_link_t *link);
uint16_t scbp_link_get_rec(const scbp_link_t *link);
scbp_link_state_t scbp_link_get_state(const scbp_link_t *link);

#ifdef __cplusplus
}
#endif

#endif /* SCBP_LINK_H */
