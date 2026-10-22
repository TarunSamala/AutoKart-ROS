#include "guard.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static ak_vcu_guard_t ready(void) {
    ak_vcu_guard_t g;
    ak_vcu_init(&g, 42);
    ak_vcu_set_limits(&g, 2.0f, 0.3f);
    ak_vcu_update_hardware(&g, AK_VCU_REQUIRED_HARDWARE, true, 1000);
    assert(ak_vcu_bind_session(&g, 71, 1000));
    assert(ak_vcu_try_ready(&g, 1000));
    return g;
}

static ak_vcu_command_t command(uint64_t seq, float speed) {
    ak_vcu_command_t c = {AK_VCU_PROTOCOL_VERSION, 42, 71, seq, 100,
                          speed, 0.0f, false};
    return c;
}

static ak_vcu_guard_t moving(void) {
    ak_vcu_guard_t g = ready();
    ak_vcu_command_t zero = command(1, 0.0f);
    assert(ak_vcu_accept(&g, &zero, 1000));
    assert(!ak_vcu_try_start(&g, false, 1000));
    assert(ak_vcu_try_start(&g, true, 1000));
    ak_vcu_command_t motion = command(2, 1.0f);
    assert(ak_vcu_accept(&g, &motion, 1010));
    return g;
}

int main(void) {
    ak_vcu_guard_t g;
    ak_vcu_init(&g, 42);
    assert(g.mode == AK_VCU_SAFE && !g.have_command);
    assert(!ak_vcu_try_ready(&g, 0));
    assert(!ak_vcu_bind_session(&g, 0, 0));
    ak_vcu_set_limits(&g, 2.0f, 0.3f);
    ak_vcu_update_hardware(&g, AK_VCU_REQUIRED_HARDWARE & ~AK_VCU_BRAKE_FEEDBACK,
                           true, 1);
    assert(ak_vcu_bind_session(&g, 71, 1));
    assert(!ak_vcu_try_ready(&g, 1));
    ak_vcu_update_hardware(&g, AK_VCU_REQUIRED_HARDWARE, true, 2);
    assert(ak_vcu_try_ready(&g, 2));
    assert(!ak_vcu_try_start(&g, true, 2));

    g = ready();
    ak_vcu_command_t off_center = command(1, 0.0f);
    off_center.steering_target_rad = 0.1f;
    assert(!ak_vcu_accept(&g, &off_center, 1000));
    assert(g.mode == AK_VCU_FAULT_LOCKOUT);
    assert(!ak_vcu_try_start(&g, true, 1000));

    g = ready();
    ak_vcu_command_t early_motion = command(1, 1.0f);
    assert(!ak_vcu_accept(&g, &early_motion, 1000));
    assert(g.mode == AK_VCU_FAULT_LOCKOUT);

    g = ready();
    ak_vcu_command_t wrong_boot = command(1, 0.0f);
    wrong_boot.boot_id = 43;
    assert(!ak_vcu_accept(&g, &wrong_boot, 1000));
    assert(g.mode == AK_VCU_FAULT_LOCKOUT);

    g = ready();
    ak_vcu_command_t over_speed = command(1, 2.1f);
    assert(!ak_vcu_accept(&g, &over_speed, 1000));
    assert(g.mode == AK_VCU_FAULT_LOCKOUT);

    g = ready();
    ak_vcu_update_hardware(&g, AK_VCU_REQUIRED_HARDWARE, false, 1001);
    assert(g.mode == AK_VCU_FAULT_LOCKOUT);

    g = moving();
    assert(g.mode == AK_VCU_AUTONOMOUS && g.target_speed_mps == 1.0f);
    ak_vcu_update_hardware(&g, AK_VCU_REQUIRED_HARDWARE, true, 1100);
    ak_vcu_tick(&g, 1109);
    assert(g.mode == AK_VCU_AUTONOMOUS);
    ak_vcu_tick(&g, 1110);
    assert(g.mode == AK_VCU_FAULT_LOCKOUT && !g.have_command &&
           g.target_speed_mps == 0.0f && (g.fault_bits & AK_VCU_FAULT_TIMEOUT));

    g = moving();
    ak_vcu_tick(&g, 1035);
    assert(g.mode == AK_VCU_FAULT_LOCKOUT && (g.fault_bits & AK_VCU_FAULT_HARDWARE));

    g = moving();
    ak_vcu_command_t replay = command(2, 1.0f);
    assert(!ak_vcu_accept(&g, &replay, 1020));
    assert(g.mode == AK_VCU_FAULT_LOCKOUT && !g.have_command);

    g = moving();
    ak_vcu_command_t wrong_session = command(3, 1.0f);
    wrong_session.session_id = 72;
    assert(!ak_vcu_accept(&g, &wrong_session, 1020));
    assert(g.fault_bits & AK_VCU_FAULT_COMMAND);

    g = moving();
    ak_vcu_command_t bad_number = command(3, NAN);
    assert(!ak_vcu_accept(&g, &bad_number, 1020));
    assert(g.target_speed_mps == 0.0f);

    g = moving();
    ak_vcu_command_t excessive_ttl = command(3, 1.0f);
    excessive_ttl.valid_for_ms = 101;
    assert(!ak_vcu_accept(&g, &excessive_ttl, 1020));

    g = moving();
    ak_vcu_update_hardware(&g, AK_VCU_REQUIRED_HARDWARE & ~AK_VCU_INDEPENDENT_STOP,
                           true, 1020);
    assert(g.mode == AK_VCU_FAULT_LOCKOUT && !g.have_command);

    g = moving();
    ak_vcu_agent_lost(&g);
    assert(g.mode == AK_VCU_FAULT_LOCKOUT && (g.fault_bits & AK_VCU_FAULT_AGENT));

    g = moving();
    ak_vcu_tick(&g, 1009);
    assert(g.mode == AK_VCU_FAULT_LOCKOUT && (g.fault_bits & AK_VCU_FAULT_CLOCK));

    g = moving();
    ak_vcu_command_t stop = command(3, 1.0f);
    stop.stop_request = true;
    assert(ak_vcu_accept(&g, &stop, 1020));
    assert(g.mode == AK_VCU_STOP_PENDING && !g.have_command &&
           g.target_speed_mps == 0.0f);
    assert(!ak_vcu_bind_session(&g, 72, 1021));
    assert(!ak_vcu_try_ready(&g, 1021));

    puts("VCU guard: adversarial host checks passed");
    return 0;
}
