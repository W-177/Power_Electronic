

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


void intgLimiter(float, float);



struct PIDController
{
    float Kp;
    float Ki;
    float Kd;
    float P;
    float I;
    float D;
    float Ctrl;
    float err;
    float errSum;
    float errRMS;
    void (*init)();
    void (*intgLimiter)(float, float);
    void (*CtrlP)(float);
    void (*CtrlI)(float);
    void (*CtrlD)(float);
    void (*CtrlPI)(float);
    void (*CtrlPD)(float);
    void (*CtrlPID)(float);
};

typedef struct PIDController PIDController;

/*
void intgLimiter(float MinLim, float MaxLim)
{

    PIDController->errSum = MaxLim * (errSum > MaxLim) + MinLim * (errSum < MinLim) + PIDController->errSum * ((errSum <= MaxLim) & (errSum => MinLim));
    /*
    if (errSum > MaxLim)
    {
        errSum = MaxLim;
    }
    else if (errSum < MinLim)
    {
        errSum = MinLim;
    }
    //else{}
    / *
}
*/













