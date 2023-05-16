#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define     R1      10e3
#define     R2      10e3
#define     R3      30e3
#define     R4      51e3
#define     C1      470e-9
#define     C2      470e-9
#define     Vref    25
#define     G       (1 + R4/R3)
#define     f       10e3


#define     iter    5000

//control
#define     kp      1.08
#define     ki      33.75



void ODE(float*, float, float, float*);
void DFxy(float*, float*);

//float data[iter];
float K1, K2, Vin=0, err=0, errSum=0;

int main()
{
    float X[2], Y[2];
    FILE *fptr;

    fptr=fopen("Datos.dat", "w+");
    if(fptr==NULL)
    {
        printf("No se pudo crear el archivo \n");
        exit(-1);
    }

    K2 = -1/(R1*R2*C1*C2);
    K1 = ((R1 + R2)*C1 + R1*C2*(1 - G))*K2;

    for (int i = 0; i < iter; i++)
    {
        err = Vref - X[1];
        errSum = errSum + err;
        Vin = kp * err + ki * errSum * 1/f;
        ODE(Y, i/f, (i + 1)/f, X);
        X[0] = Y[0];
        X[1] = Y[1];
        //data[i] = Y[1];
        fprintf(fptr, "%f %e %f\n", i/f, Y[1], Vin);
    }

    return 0;

}


void ODE(float *y, float t0, float t1, float *x)
{

    float diff[2];
    DFxy(diff, x);

    y[0] = x[0] + diff[0] * (t1 - t0);
    y[1] = x[1] + diff[1] * (t1 - t0);

}


void DFxy(float *y, float *x)
{

    y[0] = K1*x[0] + K2*x[1] - G*K2*Vin;
    y[1] = x[0];

}
