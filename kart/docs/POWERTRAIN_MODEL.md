# Powertrain Model

**Verdict: PROTOTYPE_TEST_REQUIRED.** `BldcPowertrain` calculates motor speed from a fixed provisional 5:1 ratio, bounds current by back-EMF and bus voltage, applies torque slew limits, and derives `Tchain = Tmotor*i*efficiency`. Motor electrical losses include `I²R`; controller loss is a proportional reduced-order approximation. Implemented faults include torque loss, controller disable, stuck positive torque, wrong sign, and chain break.

The torque-speed curve, ratio, efficiency, controller timing, current limits, and thermal constants require bench measurement. Regeneration, chain compliance/backlash, and motor shaft inertia are deferred.
