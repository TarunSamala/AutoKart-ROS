#include "guard.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

static void clear_command(ak_vcu_guard_t *guard) {
    guard->have_command = false;
    guard->target_speed_mps = 0.0f;
    guard->steering_target_rad = 0.0f;
}

static void fault(ak_vcu_guard_t *guard, uint32_t bits) {
    guard->fault_bits |= bits;
    guard->mode = AK_VCU_FAULT_LOCKOUT;
    clear_command(guard);
}

static bool time_ok(ak_vcu_guard_t *guard, uint64_t now_ms) {
    if (now_ms < guard->last_now_ms) {
        fault(guard, AK_VCU_FAULT_CLOCK);
        return false;
    }
    guard->last_now_ms = now_ms;
    return true;
}

static bool hardware_fresh(const ak_vcu_guard_t *guard, uint64_t now_ms) {
    return guard->have_hardware && now_ms >= guard->last_hardware_ms &&
           now_ms - guard->last_hardware_ms < AK_VCU_MAX_HARDWARE_AGE_MS;
}

void ak_vcu_init(ak_vcu_guard_t *guard, uint32_t boot_id) {
    memset(guard, 0, sizeof(*guard));
    guard->boot_id = boot_id;
    guard->mode = AK_VCU_SAFE;
}

void ak_vcu_set_limits(ak_vcu_guard_t *guard, float max_speed_mps,
                       float max_steering_rad) {
    if (guard->mode != AK_VCU_SAFE || !isfinite(max_speed_mps) ||
        !isfinite(max_steering_rad) || max_speed_mps <= 0.0f ||
        max_steering_rad <= 0.0f) {
        fault(guard, AK_VCU_FAULT_HARDWARE);
        return;
    }
    guard->max_speed_mps = max_speed_mps;
    guard->max_steering_rad = max_steering_rad;
}

void ak_vcu_update_hardware(ak_vcu_guard_t *guard, uint32_t hardware_ok,
                            bool physical_authorized, uint64_t now_ms) {
    if (!time_ok(guard, now_ms)) return;
    guard->hardware_ok = hardware_ok & AK_VCU_REQUIRED_HARDWARE;
    guard->last_hardware_ms = now_ms;
    guard->have_hardware = true;
    guard->physical_authorized = physical_authorized;
    if ((guard->mode == AK_VCU_READY || guard->mode == AK_VCU_AUTONOMOUS) &&
        (guard->hardware_ok != AK_VCU_REQUIRED_HARDWARE ||
         !guard->physical_authorized)) {
        fault(guard, AK_VCU_FAULT_HARDWARE);
    }
}

bool ak_vcu_bind_session(ak_vcu_guard_t *guard, uint64_t session_id,
                         uint64_t now_ms) {
    if (!time_ok(guard, now_ms) || guard->mode != AK_VCU_SAFE ||
        guard->boot_id == 0 || session_id == 0) return false;
    guard->session_id = session_id;
    guard->last_sequence = 0;
    guard->have_sequence = false;
    guard->agent_online = true;
    clear_command(guard);
    return true;
}

bool ak_vcu_try_ready(ak_vcu_guard_t *guard, uint64_t now_ms) {
    if (!time_ok(guard, now_ms)) return false;
    if (guard->mode != AK_VCU_SAFE || guard->fault_bits != 0 ||
        guard->hardware_ok != AK_VCU_REQUIRED_HARDWARE ||
        !guard->physical_authorized || !guard->agent_online ||
        !hardware_fresh(guard, now_ms) ||
        guard->session_id == 0 || guard->max_speed_mps <= 0.0f ||
        guard->max_steering_rad <= 0.0f) return false;
    guard->mode = AK_VCU_READY;
    return true;
}

bool ak_vcu_has_fresh_command(const ak_vcu_guard_t *guard, uint64_t now_ms) {
    return guard->have_command && now_ms >= guard->last_now_ms &&
           now_ms < guard->deadline_ms;
}

bool ak_vcu_accept(ak_vcu_guard_t *guard, const ak_vcu_command_t *command,
                   uint64_t now_ms) {
    if (!time_ok(guard, now_ms)) return false;
    if (guard->mode != AK_VCU_READY && guard->mode != AK_VCU_AUTONOMOUS)
        return false;
    if (command == NULL || command->protocol_version != AK_VCU_PROTOCOL_VERSION ||
        command->boot_id != guard->boot_id ||
        command->session_id != guard->session_id ||
        command->sequence == 0 ||
        (guard->have_sequence && command->sequence <= guard->last_sequence) ||
        command->valid_for_ms == 0 ||
        command->valid_for_ms > AK_VCU_MAX_COMMAND_AGE_MS ||
        now_ms > UINT64_MAX - command->valid_for_ms ||
        !isfinite(command->target_speed_mps) ||
        !isfinite(command->steering_target_rad) ||
        command->target_speed_mps < 0.0f ||
        command->target_speed_mps > guard->max_speed_mps ||
        fabsf(command->steering_target_rad) > guard->max_steering_rad ||
        (guard->mode == AK_VCU_READY &&
         (command->target_speed_mps != 0.0f ||
          command->steering_target_rad != 0.0f)) ||
        guard->hardware_ok != AK_VCU_REQUIRED_HARDWARE ||
        !guard->physical_authorized || !guard->agent_online ||
        !hardware_fresh(guard, now_ms)) {
        fault(guard, AK_VCU_FAULT_COMMAND);
        return false;
    }
    guard->last_sequence = command->sequence;
    guard->have_sequence = true;
    guard->deadline_ms = now_ms + command->valid_for_ms;
    guard->have_command = true;
    guard->target_speed_mps = command->stop_request ? 0.0f : command->target_speed_mps;
    guard->steering_target_rad = command->stop_request ? 0.0f : command->steering_target_rad;
    if (command->stop_request) {
        clear_command(guard);
        /* SAFE requires measured stopping proof, which this module cannot provide. */
        guard->mode = AK_VCU_STOP_PENDING;
        guard->session_id = 0;
        guard->agent_online = false;
    }
    return true;
}

bool ak_vcu_try_start(ak_vcu_guard_t *guard, bool physical_start_edge,
                      uint64_t now_ms) {
    if (!time_ok(guard, now_ms)) return false;
    if (guard->mode != AK_VCU_READY || !physical_start_edge ||
        !ak_vcu_has_fresh_command(guard, now_ms) ||
        guard->target_speed_mps != 0.0f ||
        guard->steering_target_rad != 0.0f ||
        guard->hardware_ok != AK_VCU_REQUIRED_HARDWARE ||
        !guard->physical_authorized || !hardware_fresh(guard, now_ms)) return false;
    guard->mode = AK_VCU_AUTONOMOUS;
    return true;
}

void ak_vcu_tick(ak_vcu_guard_t *guard, uint64_t now_ms) {
    if (!time_ok(guard, now_ms)) return;
    if ((guard->mode == AK_VCU_READY || guard->mode == AK_VCU_AUTONOMOUS) &&
        !hardware_fresh(guard, now_ms)) {
        fault(guard, AK_VCU_FAULT_HARDWARE);
        return;
    }
    if (guard->mode == AK_VCU_AUTONOMOUS &&
        !ak_vcu_has_fresh_command(guard, now_ms)) {
        fault(guard, AK_VCU_FAULT_TIMEOUT);
    } else if (guard->mode == AK_VCU_READY && guard->have_command &&
               !ak_vcu_has_fresh_command(guard, now_ms)) {
        clear_command(guard);
    }
}

void ak_vcu_agent_lost(ak_vcu_guard_t *guard) {
    guard->agent_online = false;
    if (guard->mode == AK_VCU_READY || guard->mode == AK_VCU_AUTONOMOUS)
        fault(guard, AK_VCU_FAULT_AGENT);
    else clear_command(guard);
}
