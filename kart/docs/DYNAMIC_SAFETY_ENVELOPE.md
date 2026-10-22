# Dynamic Safety Envelope

The prototype uses `d_stop=v*t_delay+v²/(2*a_guaranteed)+d_uncertainty+d_margin` and the positive root for clear-distance speed. Curvature applies `sqrt(a_y/abs(kappa))`. No track, actuator-health, sensor-age, or surface coupling is implemented yet.
