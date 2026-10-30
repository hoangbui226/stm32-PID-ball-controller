#ifndef IK_H
#define IK_H

/* theta_deg is the lean amount, phi_deg is the lean direction.
   deg1, deg2, deg3 are the horn commands for the 120, +x, and 240 arms.
   Returns 1 when the pose is reachable. */
int ik_solve(float theta_deg, float phi_deg, float *deg1, float *deg2, float *deg3);

#endif
