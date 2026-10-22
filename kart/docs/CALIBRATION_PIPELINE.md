# Calibration pipeline
Raw immutable files -> schema/units validation -> recorded clock alignment -> calibration split -> bounded fit -> held-out validation -> versioned report. `kart_sim.calibration` supplies CSV import/checksum and a synthetic bounded gain recovery test only; it does not fit real kart data.
