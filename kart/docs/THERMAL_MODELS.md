# Thermal Models

Every implemented thermal component uses `C*dT/dt = Ploss - (T-Tambient)/R`. Motor, controller, battery, steering actuator, brake actuator, and brake disc have first-order states. Derating is continuous between configured derate and shutdown temperatures; restart requires cooling below a hysteresis threshold. Tests cover heating, cooling, derating, and shutdown behavior of the common model.

Thermal constants, disc heat split, brake fade, airflow, and ambient variation are provisional and unvalidated.
