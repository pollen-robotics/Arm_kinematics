#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

extern "C" void setup(void);
extern "C" int forward(double *q, int n, double *M);
extern "C" int inverse(double *M, double *q);

#endif // ARM_KINEMATICS