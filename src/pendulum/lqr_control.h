#ifndef __LQR_CONTROL
#define __LQR_CONTROL

#include <gflags/gflags.h>

DECLARE_double(dt);
DECLARE_double(realtime);
DECLARE_double(simtime);
DECLARE_bool(pid);
DECLARE_double(theta);
DECLARE_bool(pub);
DECLARE_bool(real);
DECLARE_double(mass);
DECLARE_double(length);
DECLARE_double(damping);
DECLARE_double(init_pos);
DECLARE_double(Q1);
DECLARE_double(Q2);

#endif