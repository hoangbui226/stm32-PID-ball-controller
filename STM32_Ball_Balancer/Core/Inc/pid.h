#ifndef PID_H
#define PID_H

/* err_x and err_y are pixels from the plate center, after the deadband.
   theta_deg is the lean amount and phi_deg is its direction, both in degrees.
   ik_solve consumes those two angles. */
void pid(float err_x, float err_y, float *theta_deg, float *phi_deg);

#endif
