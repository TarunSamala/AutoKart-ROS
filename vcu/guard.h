#ifndef AUTOKART_VCU_GUARD_H
#define AUTOKART_VCU_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#define AK_VCU_PROTOCOL_VERSION 1u
#define AK_VCU_MAX_COMMAND_AGE_MS 100u
#define AK_VCU_MAX_HARDWARE_AGE_MS 25u

enum {
    AK_VCU_BRAKE_FEEDBACK = 1u << 0,
    AK_VCU_STEERING_FEEDBACK = 1u << 1,
    AK_VCU_SPEED_FEEDBACK = 1u << 2,
    AK_VCU_INDEPENDENT_STOP = 1u << 3,
    AK_VCU_TRACTION_INHIBIT = 1u << 4,
    AK_VCU_ESTOP_HEALTHY = 1u << 5,
    AK_VCU_DRIVER_HEALTHY = 1u << 6,
    AK_VCU_CALIBRATION_VALID = 1u << 7,
    AK_VCU_REQUIRED_HARDWARE = 0xffu
};

typedef enum {
    AK_VCU_SAFE = 0,
    AK_VCU_READY,
    AK_VCU_AUTONOMOUS,
    AK_VCU_STOP_PENDING,
    AK_VCU_FAULT_LOCKOUT
} ak_vcu_mode_t;

typedef enum {
    AK_VCU_FAULT_NONE = 0,
    AK_VCU_FAULT_HARDWARE = 1u << 0,
    AK_VCU_FAULT_COMMAND = 1u << 1,
    AK_VCU_FAULT_TIMEOUT = 1u << 2,
    AK_VCU_FAULT_CLOCK = 1u << 3,
    AK_VCU_FAULT_AGENT = 1u << 4
} ak_vcu_fault_t;

typedef struct {
    uint16_t protocol_version;
    uint32_t boot_id;
    uint64_t session_id;
    uint64_t sequence;
    uint32_t valid_for_ms;
    float target_speed_mps;
    float steering_target_rad;
    bool stop_request;
} ak_vcu_command_t;

typedef struct {
    ak_vcu_mode_t mode;
    uint32_t fault_bits;
    uint32_t hardware_ok;
    uint32_t boot_id;
    uint64_t session_id;
    uint64_t last_sequence;
    uint64_t last_now_ms;
    uint64_t last_hardware_ms;
    uint64_t deadline_ms;
    float max_speed_mps;
    float max_steering_rad;
    float target_speed_mps;
    float steering_target_rad;
    bool have_command;
    bool have_sequence;
    bool have_hardware;
    bool physical_authorized;
    bool agent_online;
} ak_vcu_guard_t;

void ak_vcu_init(ak_vcu_guard_t *guard, uint32_t boot_id);
void ak_vcu_set_limits(ak_vcu_guard_t *guard, float max_speed_mps,
                       float max_steering_rad);
void ak_vcu_update_hardware(ak_vcu_guard_t *guard, uint32_t hardware_ok,
                            bool physical_authorized, uint64_t now_ms);
bool ak_vcu_bind_session(ak_vcu_guard_t *guard, uint64_t session_id,
                         uint64_t now_ms);
bool ak_vcu_try_ready(ak_vcu_guard_t *guard, uint64_t now_ms);
bool ak_vcu_accept(ak_vcu_guard_t *guard, const ak_vcu_command_t *command,
                   uint64_t now_ms);
/* physical_start_edge must be a fresh edge from the physical input after READY. */
bool ak_vcu_try_start(ak_vcu_guard_t *guard, bool physical_start_edge,
                      uint64_t now_ms);
void ak_vcu_tick(ak_vcu_guard_t *guard, uint64_t now_ms);
void ak_vcu_agent_lost(ak_vcu_guard_t *guard);
bool ak_vcu_has_fresh_command(const ak_vcu_guard_t *guard, uint64_t now_ms);

#endif
