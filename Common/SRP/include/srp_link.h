#ifndef SRP_LINK_H
#define SRP_LINK_H

#include <stdint.h>

#include "srp_codec.h"
#include "srp_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SRP_LINK_PENDING_SLOTS UINT8_C(4)
#define SRP_LINK_ACK_TIMEOUT_MS UINT32_C(500)
#define SRP_LINK_MAX_RETRIES UINT8_C(3)

typedef enum {
    SRP_LINK_ACTIVE = 0,
    SRP_LINK_WARNING,
    SRP_LINK_PASSIVE,
    SRP_LINK_BUS_OFF
} srp_link_state_t;

typedef enum {
    SRP_LINK_TX_OK = 0,
    SRP_LINK_TX_TRANSPORT_FAILURE,
    SRP_LINK_TX_TIMEOUT,
    SRP_LINK_TX_REMOTE_ERROR,
    SRP_LINK_TX_BUS_OFF
} srp_link_tx_result_t;

typedef int (*srp_link_transport_send_t)(const uint8_t *data, uint16_t length,
                                         void *context);
typedef void (*srp_link_tx_callback_t)(srp_link_tx_result_t result,
                                       uint8_t status_code, void *context);
typedef void (*srp_link_frame_callback_t)(const srp_frame_t *frame,
                                          void *context);
typedef void (*srp_link_bus_off_callback_t)(void *context);

#pragma pack(push, 4)
typedef struct {
    uint8_t local_node;
    uint32_t ack_timeout_ms;
    uint8_t max_retries;
    srp_link_transport_send_t transport_send;
    srp_link_frame_callback_t on_frame;
    srp_link_bus_off_callback_t on_bus_off;
    void *context;
} srp_link_config_t;

typedef struct {
    uint8_t in_use;
    uint8_t retry_count;
    uint8_t type;
    uint8_t sequence;
    uint16_t frame_length;
    uint32_t last_tx_ms;
    srp_link_tx_callback_t callback;
    void *callback_context;
    uint8_t frame_bytes[SRP_MAX_FRAME_SIZE];
} srp_link_pending_t;

typedef struct {
    uint16_t tec;
    uint16_t rec;
    uint8_t next_sequence;
    srp_link_state_t state;
    srp_link_config_t config;
    srp_link_pending_t pending[SRP_LINK_PENDING_SLOTS];
    /* Callers serialize srp_link_send() for this mutable, non-ACK TX buffer. */
    uint8_t tx_scratch[SRP_MAX_FRAME_SIZE];
} srp_link_t;
#pragma pack(pop)

void srp_link_init(srp_link_t *link, const srp_link_config_t *config);
int srp_link_send(srp_link_t *link, uint8_t priority, uint8_t destination,
                  uint16_t type, uint8_t flags, const uint8_t *payload,
                  uint16_t length, uint32_t now_ms,
                  srp_link_tx_callback_t callback, void *callback_context);
int srp_link_send_fast_response(srp_link_t *link, uint8_t priority,
                                uint8_t destination, uint8_t is_error,
                                uint16_t ack_type, uint8_t ack_sequence,
                                uint8_t status_code, uint32_t now_ms);
void srp_link_cancel_message(srp_link_t *link, uint16_t type);
void srp_link_receive(srp_link_t *link, const srp_frame_t *frame);
void srp_link_report_parser_error(srp_link_t *link, srp_parser_error_t error);
void srp_link_tick(srp_link_t *link, uint32_t now_ms);
void srp_link_recover(srp_link_t *link);
uint16_t srp_link_get_tec(const srp_link_t *link);
uint16_t srp_link_get_rec(const srp_link_t *link);
srp_link_state_t srp_link_get_state(const srp_link_t *link);

#ifdef __cplusplus
}
#endif

#endif /* SRP_LINK_H */
