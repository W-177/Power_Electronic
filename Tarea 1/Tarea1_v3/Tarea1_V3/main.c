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


//simulation time
#define     iter    5001

//control parameters
#define     Kp      .02
#define     Ki      15/2.7
#define     Kd      0.0


//Funciones Globales
void controlPID(float*, float);

void SystemODE(float*, float*);

//void RungeKutta();
void ODE_Euler(float*, float*, float);
void ODE_RK2(float*, float*, float);
void ODE_RK4(float*, float*, float);
void ODE_RK5(float*, float*, float);
void ODE_SRK4(float*, float*, float);
//void ODE_RKF4(float*, float*, float);
//void ODE_RKF5(float*, float*, float);


void Solve(float*, float*, float, FILE*, void (*NumericMethod)(float*, float*, float));

//Variables Globales del Sistema
//float data[iter];
float K1, K2, Vin = 0, err = 0, errSum = 0;

int main()
{

    //solver function selector
    //0:    Euler
    //1:    Runge-Kutta Orden 2
    //2:    Runge-Kutta Orden 4
    //3:    Runge-Kutta Orden 5
    //4:    Sarafyan's Runge-Kutta Orden 4
    //5:    Runge-Kutta Orden–Fehlberg 4 (Not working)
    //6:    Runge-Kutta Orden–Fehlberg 5 (Not working)
    short NMtdStr = 3;
    FILE *fptr;
    float X[2], Y[2];


    void *FileName[] = {"DatosEU.dat", "DatosRK2.dat", "DatosRK4.dat", "DatosRK5.dat", "DatosSRK4.dat"}; //, "DatosRKF4.dat", "DatosRKF5.dat"}
    fptr=fopen(FileName[NMtdStr], "w+");


    if(fptr==NULL)
    {
        printf("No se pudo crear el archivo \n");
        exit(-1);
    }

    void (*NumMethod[])(float*, float*, float) = {ODE_Euler, ODE_RK2, ODE_RK4, ODE_RK5, ODE_SRK4}; //, ODE_RKF4, ODE_RKF5};

    K2 = -1/(R1*R2*C1*C2);
    K1 = ((R1 + R2)*C1 + R1*C2*(1 - G))*K2;

    Solve(Y, X, 1/f, fptr, (*NumMethod[NMtdStr]));

    printf("\nVsalida: %e\nVref: %d\nVin: %f\nerr: %e\nerrSum: %e\nh: %e\n\n", G*Y[1], Vref, Vin, err, errSum, 1/f);

    return 0;
}



//Función Principal de solución del sistema
void Solve(float *y, float *x, float h, FILE *fptr, void (*NumericMethod)(float*, float*, float))
{
    for(int i=0; i < iter; i++)
    {
        controlPID(x, h);
        (*NumericMethod)(y, x, h);

        x[0] = y[0];
        x[1] = y[1];

        //data[i] = y[1];
        fprintf(fptr, "%f  %e\n", i*h, G*y[1]);
    }

}



//  Ecuaciones del sistema
void SystemODE(float *y, float *x)
{
    y[0] = K1*x[0] + K2*(x[1] - Vin);
    y[1] = x[0];
}



//  Controlador PID del sistema
void controlPID(float *x, float h)
{
    float erAux = err;
    err = Vref - G*x[1];
    errSum = errSum + err;
    Vin = Kp * err + Ki * (errSum * h) + Kd * (err - erAux)/h;
}

/*
void ControlLimiter()
{

}
*/

/*  Métodos Muméricos para ODEs     */

//  0:  Euler
void ODE_Euler(float *y, float *x, float h)
{

    float diff[2];
    SystemODE(diff, x);

    y[0] = x[0] + diff[0] * h;
    y[1] = x[1] + diff[1] * h;

}


//  1:  Runge-Kutta 2th Order
void ODE_RK2(float *y, float *x, float h)
{

    float k1[2], k2[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h/2;
    xAux[1] = x[1] + k1[1] * h/2;
    SystemODE(k2, xAux);

    y[0] = x[0] + h * k2[0];
    y[1] = x[1] + h * k2[1];

}



//  2:  Runge-Kutta 4th Order
void ODE_RK4(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h/2;
    xAux[1] = x[1] + k1[1] * h/2;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + k2[0] * h/2;
    xAux[1] = x[1] + k2[1] * h/2;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + k3[0] * h;
    xAux[1] = x[1] + k3[1] * h;
    SystemODE(k4, xAux);

    y[0] = x[0] + h/6 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
    y[1] = x[1] + h/6 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);

}


//  3:  RungeKutta 5th Order
void ODE_RK5(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h/4;
    xAux[1] = x[1] + k1[1] * h/4;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + k1[0] * h/8 + k2[0] * h * 1/8;
    xAux[1] = x[1] + k1[1] * h/8 + k2[1] * h * 1/8;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + k1[0] * h * 0 - k2[0] * h * 1/2 + k3[0] * h;
    xAux[1] = x[1] + k1[1] * h * 0 - k2[1] * h * 1/2 + k3[1] * h;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + k1[0] * h * 3/16 - k2[0] * h * 0 + k3[0] * h * 0 - k4[0] * h * 9/16;
    xAux[1] = x[1] + k1[1] * h * 3/16 - k2[1] * h * 0 + k3[1] * h * 0 - k4[1] * h * 9/16;
    SystemODE(k5, xAux);

    xAux[0] = x[0] - k1[0] * h * 3/7 + k2[0] * h * 2/7 + k3[0] * h * 12/7 - k4[0] * h * 12/7 + k5[0] * h * 8/7;
    xAux[1] = x[1] - k1[1] * h * 3/7 + k2[1] * h * 2/7 + k3[1] * h * 12/7 - k4[1] * h * 12/7 + k5[1] * h * 8/7;
    SystemODE(k6, xAux);

    y[0] = x[0] + h/90 * (7*k1[0] + 32*k2[0] + 12*k3[0] + 32*k4[0] + 7*k5[0]);
    y[1] = x[1] + h/90 * (7*k1[1] + 32*k2[1] + 12*k3[1] + 32*k4[1] + 7*k5[1]);

}

//  4:    Sarafyan's RK4 (Runge–Kutta–Fehlberg method)
void ODE_SRK4(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], xAux[2];//, k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h/2.0;
    xAux[1] = x[1] + k1[1] * h/2.0;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + k1[0] * h/4.0 + k2[0] * h/4.0;
    xAux[1] = x[1] + k1[1] * h/4.0 + k2[1] * h/4.0;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + k1[0] * h * 0.0 - k2[0] * h * 1.0 + k3[0] * h * 2.0;
    xAux[1] = x[1] + k1[1] * h * 0.0 - k2[1] * h * 1.0 + k3[1] * h * 2.0;
    SystemODE(k4, xAux);

    //xAux[0] = x[0] + k1[0] * h * 7.0/27.0 - k2[0] * h * 10.0/27.0 + k3[0] * h * 0.0 - k4[0] * h * 1.0/27.0;
    //xAux[1] = x[1] + k1[1] * h * 7.0/27.0 - k2[1] * h * 10.0/27.0 + k3[1] * h * 0.0 - k4[1] * h * 1.0/27.0;
    //SystemODE(k5, xAux);

    //xAux[0] = x[0] + k1[0] * h * 28.0/625.0 - k2[0] * h * 1.0/5. + k3[0] * h * 546.0/625.0 - k4[0] * h * 54.0/625.0 - k5[0] * h * 378.0/625.0;
    //xAux[1] = x[1] + k1[1] * h * 28.0/625.0 - k2[1] * h * 1.0/5. + k3[1] * h * 546.0/625.0 - k4[1] * h * 54.0/625.0 - k5[1] * h * 378.0/625.0;
    //SystemODE(k6, xAux);

    y[0] = x[0] + h/6 * (k1[0] + 0*k2[0] + 4*k3[0] + k4[0]);
    y[1] = x[1] + h/6 * (k1[1] + 0*k2[1] + 4*k3[1] + k4[1]);

    //y[0] = x[0] + h * (1.0/24.0*k1[0] + 0.0*k2[0] + 0.0*k3[0] + 5.0/48.0*k4[0] + 27.0/56.0*k5[0] + 125.0/336.0*k6[0]);
    //y[1] = x[1] + h * (1.0/24.0*k1[1] + 0.0*k2[1] + 0.0*k3[1] + 5.0/48.0*k4[1] + 27.0/56.0*k5[0] + 125.0/336.0*k6[0]);

}

//LOS SIGUIENTES MÉTODOS TIENEN PROBLEMAS CON LAS DIVISIONES DE VARIOS DE LOS COEFICIENTES
//HAY QUE REDEFINIR TODAS LAS VARIABLES COMO DOUBLES.

/*

/////////////////
//  5:   Runge–Kutta–Fehlberg method
void ODE_RKF4(float *y, float *x, float h)
{


    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h * 2/9;
    xAux[1] = x[1] + k1[1] * h * 2/9;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + k1[0] * h * 1/12 + k2[0] * h * 1/4;
    xAux[1] = x[1] + k1[1] * h * 1/12 + k2[1] * h * 1/4;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + k1[0] * h * 69/128 - k2[0] * h * 243/128 + k3[0] * h * 135/64;
    xAux[1] = x[1] + k1[1] * h * 69/128 - k2[1] * h * 243/128 + k3[1] * h * 135/64;
    SystemODE(k4, xAux);

    xAux[0] = x[0] - k1[0] * h * 17/12 + k2[0] * h * 27/4 - k3[0] * h * 27/5 + k4[0] * h * 16/15;
    xAux[1] = x[1] - k1[1] * h * 17/12 + k2[1] * h * 27/4 - k3[1] * h * 27/5 + k4[1] * h * 16/15;
    SystemODE(k5, xAux);

    xAux[0] = x[0] + k1[0] * h * 65/432 - k2[0] * h * 5/16 + k3[0] * h * 13/16 + k4[0] * h * 4/27 + k5[0] * h * 5/144;
    xAux[1] = x[1] + k1[1] * h * 65/432 - k2[1] * h * 5/16 + k3[1] * h * 13/16 + k4[1] * h * 4/27 + k5[1] * h * 5/144;
    SystemODE(k6, xAux);

    //y[0] = x[0] + h * (1/9*k1[0] + 0*k2[0] + 9/20*k3[0] + 16/45*k4[0] + 1/12*k5[0]);
    //y[1] = x[1] + h * (1/9*k1[1] + 0*k2[1] + 9/20*k3[1] + 16/45*k4[1] + 1/12*k5[1]);
    y[0] = x[0] + h * (47/450*k1[0] + 0*k2[0] + 12/25*k3[0] + 32/225*k4[0] + 1/30*k5[0] + 6/25*k5[0]);
    y[1] = x[1] + h * (47/450*k1[1] + 0*k2[1] + 12/25*k3[1] + 32/225*k4[1] + 1/30*k5[1] + 6/25*k5[1]);



    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h/4;
    xAux[1] = x[1] + k1[1] * h/4;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + k1[0] * h * 3/32 + k2[0] * h * 9/32;
    xAux[1] = x[1] + k1[1] * h * 3/32 + k2[1] * h * 9/32;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + k1[0] * h * 1932/2197 - k2[0] * h * 7200/2197 + k3[0] * h * 7296/2197;
    xAux[1] = x[1] + k1[1] * h * 1932/2197 - k2[1] * h * 7200/2197 + k3[1] * h * 7296/2197;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + k1[0] * h * 439/216 - k2[0] * h * 8 + k3[0] * h * 3680/513 - k4[0] * h * 845/4104;
    xAux[1] = x[1] + k1[1] * h * 439/216 - k2[1] * h * 8 + k3[1] * h * 3680/513 - k4[1] * h * 845/4104;
    SystemODE(k5, xAux);

    xAux[0] = x[0] - k1[0] * h * 8/27 + k2[0] * h * 2 - k3[0] * h * 3544/2565 + k4[0] * h * 1859/4104 - k5[0] * h * 11/40;
    xAux[1] = x[1] - k1[1] * h * 8/27 + k2[1] * h * 2 - k3[1] * h * 3544/2565 + k4[1] * h * 1859/4104 - k5[1] * h * 11/40;
    SystemODE(k6, xAux);

    y[0] = x[0] + h * (25/216*k1[0] + 0*k2[0] + 1408/2565*k3[0] + 2197/4104*k4[0] - 1/5*k5[0] + 0*k6[0]);
    y[1] = x[1] + h * (25/216*k1[1] + 0*k2[1] + 1408/2565*k3[1] + 2197/4104*k4[1] - 1/5*k5[1] + 0*k6[0]);

}

////////////////
//  6:   Runge–Kutta–Fehlberg method
void ODE_RKF5(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h/4;
    xAux[1] = x[1] + k1[1] * h/4;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + k1[0] * h * 3/32 + k2[0] * h * 9/32;
    xAux[1] = x[1] + k1[1] * h * 3/32 + k2[1] * h * 9/32;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + k1[0] * h * 1932/2197 - k2[0] * h * 7200/2197 + k3[0] * h * 7296/2197;
    xAux[1] = x[1] + k1[1] * h * 1932/2197 - k2[1] * h * 7200/2197 + k3[1] * h * 7296/2197;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + k1[0] * h * 439/216 - k2[0] * h * 8 + k3[0] * h * 3680/513 - k4[0] * h * 845/4104;
    xAux[1] = x[1] + k1[1] * h * 439/216 - k2[1] * h * 8 + k3[1] * h * 3680/513 - k4[1] * h * 845/4104;
    SystemODE(k5, xAux);

    xAux[0] = x[0] - k1[0] * h * 8/27 + k2[0] * h * 2 - k3[0] * h * 3544/2565 + k4[0] * h * 1859/4104 - k5[0] * h * 11/40;
    xAux[1] = x[1] - k1[1] * h * 8/27 + k2[1] * h * 2 - k3[1] * h * 3544/2565 + k4[1] * h * 1859/4104 - k5[1] * h * 11/40;
    SystemODE(k6, xAux);

    y[0] = x[0] + h * (16/135*k1[0] + 0*k2[0] + 6656/12825*k3[0] + 28561/56430*k4[0] - 9/50*k5[0] + 2/55*k6[0]);
    y[1] = x[1] + h * (16/135*k1[1] + 0*k2[1] + 6656/12825*k3[1] + 28561/56430*k4[1] - 9/50*k5[1] + 2/55*k6[1]);

}

*/


/*
//typedef struct PIDController PIDController, *pno;
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
*/


