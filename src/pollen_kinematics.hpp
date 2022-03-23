#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#if defined (_WIN32)
#define EXPORT_MACRO __declspec(dllexport)
#else
#define EXPORT_MACRO 
#endif

enum ArmSide { Left, Right };

extern "C" EXPORT_MACRO void setup(void);
extern "C" EXPORT_MACRO void forward(ArmSide side, double *q, int n, double *M);
extern "C" EXPORT_MACRO void inverse(ArmSide side, double *M, double *q);

#endif // ARM_KINEMATICS