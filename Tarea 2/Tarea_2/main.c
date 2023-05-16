#include <stdio.h>
#include <stdlib.h>
#include <math.h>




#define     Ra      2.581       //ohm
#define     La      28e-3       //A
#define     Km      1.8         //N m/A
#define     Je      0.02215     //kg m^2
#define     B       0           //N m s
#define     Vref    240.0       //V
#define     fc      10e3        //Hz
#define     w       100         //rad/s
#define     fs      1e6         //Hz

//simulation time
#define     iter    10000

//cantidad de controladores
//#define     ctrlN   2


//Funciones Globales

//PID Control Inizialization

//PID Control Structure:
typedef struct PIDController
{
    float Kp;
    float Ki;
    float Kd;
    float Cout;
    float err;
    float errSum;
    struct PIDController* SELF;
    //float errRMS;
    void (*init)(struct PIDController*, float, float, float);
    void (*intgLimiter)(struct PIDController*, float, float);
    //void (*intgUpLimiter)(float, float);
    void (*ControlLimiter)(float*, float, float);
    //void (*ControlUpLimiter)(float, float);
    void (*controlPIDideal)(struct PIDController*, float, float, float*, float);
    void (*controlPIDparal)(struct PIDController*, float, float, float*, float);
    //void (*errRMScalc)(float, float, float);
}   PIDCtrl;


//Control Structure Functions Inicializator
void CtrStrFnctInit(struct PIDController*, int, int);


//Control Structure Functions:
void init(struct PIDController*, float, float, float);
void intgLimiter(struct PIDController*, float, float);
void ControlLimiter(float*, float, float);
void controlPIDideal(struct PIDController*, float, float, float*, float);
void controlPIDparal(struct PIDController*, float, float, float*, float);
void controlPID(float*, float);


//System ODE
void SystemODE(float*, float*);


//void RungeKutta();
void ODE_Euler(float*, float*, float);
void ODE_RK2(float*, float*, float);
void ODE_RK4(float*, float*, float);
void ODE_RK5(float*, float*, float);
void ODE_SRK4(float*, float*, float);
void ODE_SRK5(float*, float*, float);
void ODE_RKF4(float*, float*, float);
void ODE_RKF5(float*, float*, float);
void ODE_RK6(float*, float*, float);
void ODE_RKF7(float*, float*, float);
void ODE_HIRK6(float*, float*, float);


//System Solver
void Solve(float*, float*, float, float, FILE*, void (*NumericMethod)(float*, float*, float), struct PIDController *PID);


//Variables Globales del Sistema
float Vin = Vref, Iref = 0, d;//, err = 0, errSum = 0;



int main()
{

    //solver function selector
    //0:    Euler
    //1:    Runge-Kutta Orden 2
    //2:    Runge-Kutta Orden 4
    //3:    Runge-Kutta Orden 5
    //4:    Sarafyan's Runge-Kutta Orden 4
    //5:    Sarafyan's Runge-Kutta Orden 5
    //6:    Runge-Kutta–Fehlberg Orden 4
    //7:    Runge-Kutta–Fehlberg Orden 5
    //8:    Runge-Kutta Orden 6
    //9:    Runge-Kutta–Fehlberg Orden 7
    //10:   Huta's Improved RungeKutta 6
    short NMtdStr = 10;
    FILE *fptr;
    float X[2], Y[2];


    //File Name Selector inicialization and selection
    void *FileName[] = {"DatosEU.dat", "DatosRK2.dat", "DatosRK4.dat", "DatosRK5.dat", "DatosSRK4.dat", "DatosSRK5.dat", "DatosRKF4.dat", "DatosRKF5.dat", "DatosRK6.dat", "DatosRKF7.dat", "DatosHIRK6.dat"};
    fptr=fopen(FileName[NMtdStr], "w+");


    //PID declaration
    //0: current PID
    //1: Angular Speed PID
    int ctrlN = 2;      //cantidad de controladores
    PIDCtrl PID[ctrlN];


    //constantes del controlador

    float kp[] = {2.33, 0.108}, ki[] = {6.17, 2.28}, kd[] = {0, 0};



    //Inicialize PID Controls Methods
    CtrStrFnctInit(PID, ctrlN, 0);

    //PIDs inicialization
    for(int i = 0; i < ctrlN; i++)
    {
        PID[i].init(PID[i].SELF, kp[i], ki[i], kd[i]);
    }
    //PID[i].init(PID[i].SELF, kp[i], ki[i], kd[1]);
    //PID[i].init(PID[i].SELF, kp[i], ki[i], kd[i]);


    printf("\nPID1\nKp = %f\nKi = %f\nKd = %f\n", PID[0].Kp, PID[0].Ki, PID[0].Kd);
    printf("\nPID2\nKp = %f\nKi = %f\nKd = %f\n", PID[1].Kp, PID[1].Ki, PID[1].Kd);


    //ERROR: FILE NO EXIST -> EXIT
    if(fptr==NULL)
    {
        printf("No se pudo crear el archivo \n");
        exit(-1);
    }

    //Numerical ODE solver Selector inicialization
    void (*NumMethod[])(float*, float*, float) = {ODE_Euler, ODE_RK2, ODE_RK4, ODE_RK5, ODE_SRK4, ODE_SRK5, ODE_RKF4, ODE_RKF5, ODE_RK6, ODE_RKF7, ODE_HIRK6};


    Solve(Y, X, 1/fs, fc, fptr, (*NumMethod[NMtdStr]), PID);

    //printf("\nIout: %e\nwout: %e\nVref: %d\nVin: %f\nerrI: %e\nerrSumI: %e\nerrw: %e\nerrSumw: %e\nh: %e\n\n", Y[0], Y[1], Vref, Vin, PID[0].err, PID[0].errSum, PID[1].err, PID[1].errSum, 1/f);

    return 0;
}



//==============================================================


//Función Principal de solución del sistema

void Solve(float *y, float *x, float h, float f, FILE *fptr, void (*NumericMethod)(float*, float*, float), struct PIDController *PID)
{
    printf("\n%f\n", PID[0].err);

    for(int i=0; i < iter; i++)
    {

        PID[0].controlPIDideal(&PID[1], x[1], h, &Iref, w);
        //printf("\nIref = %f\n", Iref);
        PID[1].controlPIDideal(&PID[0], x[0], h, &d, Iref);
        //printf("\nVin = %f\n", Vin);
        PID[1].ControlLimiter(&d, 1, 0);


        for (int j = 0; j < 1/h/f ; j++)
        {

            Vin = (d >= j*h*f) * Vref;

            (*NumericMethod)(y, x, h);

            x[0] = y[0];
            x[1] = y[1];

            fprintf(fptr, "%f  %f  %e  %e\n", (i*1/h/f+j)*h, Vin, x[0], x[1]);

        }


    }
    printf("\nSalida\n   i = %e\n   w = %e\n   Vin = %e\n", 30/M_PI * x[0], x[1], Vin);
    //printf("\n%d\n", ((int)fc)%((int)fs));
    //printf("\nFF %f\n", (d >= 0*h*f) * Vref);
}


//==============================================================


//  Ecuaciones del sistema

void SystemODE(float *y, float *x)
{
    y[0] = (-Ra * x[0] - Km * x[1] + Vin) /La;
    y[1] = (Km * x[0] - B * x[1]) /Je;
}


//==============================================================


//  Controlador PID del sistema

//PID struct / attributes
/*

   // was declared up

typedef struct PIDController
{
    float Kp;
    float Ki;
    float Kd;
    float Cout;
    float err;
    float errSum;
    struct PIDController* SELF;
    //float errRMS;
    void (*init)(struct ControlLimiter*, float, float, float);
    void (*intgLimiter)(float, float, float);
    //void (*intgUpLimiter)(float, float);
    void (*ControlLimiter)(float, float, float);
    //void (*ControlUpLimiter)(float, float);
    void (*controlPIDideal)(struct ControlLimiter*, float, float, float);
    void (*controlPIDparal)(struct ControlLimiter*, float, float, float);
    //void (*errRMScalc)(float, float, float);
} PIDController;

*/

//------------------------------------------
//Control Structure Functions Inizializator:
void CtrStrFnctInit(struct PIDController *PID, int N, int j)
{

    for(int i = j; i < N; i++)
    {
        PID[i].SELF = &PID[i];
        PID[i].init = init;
        PID[i].intgLimiter = intgLimiter;
        PID[i].ControlLimiter = ControlLimiter;
        PID[i].controlPIDideal = controlPIDideal;
        PID[i].controlPIDparal = controlPIDparal;
    }

}
//------------------------------------------

//PID methods/functions

void init(struct PIDController *PID, float Kp, float Ki, float Kd)
{

    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->err = 0;
    PID->errSum = 0;
    PID->Cout = 0;

}


void controlPIDideal(struct PIDController *PID, float x, float h, float *yout, float VarRef)
{
    float erAux = PID->err;
    PID->err = VarRef - x;
    PID->errSum = PID->errSum + PID->err;
    *yout = PID->Kp * PID->err + PID->Ki * (PID->errSum * h) + PID->Kd * (PID->err - erAux)/h;
}


void controlPIDparal(struct PIDController *PID, float x, float h, float *yout, float VarRef)
{
    float erAux = PID->err;
    PID->err = VarRef - x;
    PID->errSum = PID->errSum + PID->err;
    *yout = PID->Kp * (PID->err + PID->Ki * (PID->errSum * h) + PID->Kd * (PID->err - erAux)/h);
}

/*
void controlPID(float *x, float h, yout)
{
    float erAux = err;
    err = Vref - x[1];
    errSum = errSum + err;
    yout = Kp * err + Ki * (errSum * h) + Kd * (err - erAux)/h;
}
*/


void intgLimiter(struct PIDController* PID, float UpLim, float DownLim)
{

    PID->errSum = DownLim * (PID->errSum < DownLim) + UpLim * (PID->errSum > UpLim) + PID->errSum * ((PID->errSum < UpLim) & (PID->errSum > DownLim));

}

/*
void intgUpLimiter(struct PIDController* PID, float UpLim)
{

    PID->errSum = UpLim * (PID->errSum > UpLim) + PID->errSum * (PID->errSum < UpLim);

}
*/

void ControlLimiter(float *yout, float UpLim, float DownLim)
{

    *yout = DownLim * (*yout < DownLim) + UpLim * (*yout > UpLim) + *yout * ((*yout < UpLim) & (*yout > DownLim));

}

/*
void ControlUpLimiter(float yout, float UpLim)
{

    yout = UpLim * (yout > UpLim) + yout * (yout < UpLim);

}
*/

//==============================================================


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

    xAux[0] = x[0] + k1[0] * h /2.0;
    xAux[1] = x[1] + k1[1] * h /2.0;
    SystemODE(k2, xAux);

    y[0] = x[0] + h * k2[0];
    y[1] = x[1] + h * k2[1];

}



//  2:  Runge-Kutta 4th Order
void ODE_RK4(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h /2.0;
    xAux[1] = x[1] + k1[1] * h /2.0;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + k2[0] * h /2.0;
    xAux[1] = x[1] + k2[1] * h /2.0;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + k3[0] * h;
    xAux[1] = x[1] + k3[1] * h;
    SystemODE(k4, xAux);

    y[0] = x[0] + h/6.0 * (k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0]);
    y[1] = x[1] + h/6.0 * (k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1]);

}


//  3:  RungeKutta 5th Order
void ODE_RK5(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h /4.0;
    xAux[1] = x[1] + k1[1] * h /4.0;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + (k1[0] + k2[0]) /8.0 * h;
    xAux[1] = x[1] + (k1[1] + k2[1]) /8.0 * h;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + (k1[0] * 0.0 - k2[0] * 1.0/2.0 + k3[0]) * h;
    xAux[1] = x[1] + (k1[1] * 0.0 - k2[1] * 1.0/2.0 + k3[1]) * h;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + (k1[0] * 3.0 - k2[0] * 0 + k3[0] * 0 - k4[0] * 9.0) /16.0 * h;
    xAux[1] = x[1] + (k1[1] * 3.0 - k2[1] * 0 + k3[1] * 0 - k4[1] * 9.0) /16.0 * h;
    SystemODE(k5, xAux);

    xAux[0] = x[0] - (k1[0] * 3.0 - k2[0] * 2.0 - k3[0] * 12.0 + k4[0] * 12.0 - k5[0] * 8.0) /7.0 * h;
    xAux[1] = x[1] - (k1[1] * 3.0 - k2[1] * 2.0 - k3[1] * 12.0 + k4[1] * 12.0 - k5[1] * 8.0) /7.0 * h;
    SystemODE(k6, xAux);

    y[0] = x[0] + h/90.0 * (7.0*k1[0] + 32.0*k2[0] + 12.0*k3[0] + 32.0*k4[0] + 7.0*k5[0]);
    y[1] = x[1] + h/90.0 * (7.0*k1[1] + 32.0*k2[1] + 12.0*k3[1] + 32.0*k4[1] + 7.0*k5[1]);

}

//  4:    Sarafyan's RK4 (Runge–Kutta–Fehlberg method)
void ODE_SRK4(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], xAux[2];//, k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h /2.0;
    xAux[1] = x[1] + k1[1] * h /2.0;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + (k1[0] + k2[0])/4.0 * h;
    xAux[1] = x[1] + (k1[1] + k2[1])/4.0 * h;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + (k1[0] * 0.0 - k2[0] * 1.0 + k3[0] * 2.0) * h;
    xAux[1] = x[1] + (k1[1] * 0.0 - k2[1] * 1.0 + k3[1] * 2.0) * h;
    SystemODE(k4, xAux);

    y[0] = x[0] + h/6 * (k1[0] + 0*k2[0] + 4*k3[0] + k4[0]);
    y[1] = x[1] + h/6 * (k1[1] + 0*k2[1] + 4*k3[1] + k4[1]);

}



//  5:    Sarafyan's RK5 (Runge–Kutta–Fehlberg method)
void ODE_SRK5(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h /2.0;
    xAux[1] = x[1] + k1[1] * h /2.0;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + (k1[0] + k2[0])/4.0 * h;
    xAux[1] = x[1] + (k1[1] + k2[1])/4.0 * h;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + (k1[0] * 0.0 - k2[0] + k3[0] * 2.0) * h;
    xAux[1] = x[1] + (k1[1] * 0.0 - k2[1] + k3[1] * 2.0) * h;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + (k1[0] * 7.0 + k2[0] * 10.0 + k3[0] * 0.0 + k4[0]) * h /27.0;
    xAux[1] = x[1] + (k1[1] * 7.0 + k2[1] * 10.0 + k3[1] * 0.0 + k4[1]) * h /27.0;
    SystemODE(k5, xAux);

    xAux[0] = x[0] + (k1[0] * 28.0/625.0 - k2[0] /5.0 + k3[0] * 546.0/625.0 + k4[0] * 54.0/625.0 - k5[0] * 378.0/625.0) * h;
    xAux[1] = x[1] + (k1[1] * 28.0/625.0 - k2[1] /5.0 + k3[1] * 546.0/625.0 + k4[1] * 54.0/625.0 - k5[1] * 378.0/625.0) * h;
    SystemODE(k6, xAux);

    y[0] = x[0] + h * (k1[0]/24.0 + 0.0*k2[0] + 0.0*k3[0] + 5.0/48.0*k4[0] + 27.0/56.0*k5[0] + 125.0/336.0*k6[0]);
    y[1] = x[1] + h * (k1[1]/24.0 + 0.0*k2[1] + 0.0*k3[1] + 5.0/48.0*k4[1] + 27.0/56.0*k5[1] + 125.0/336.0*k6[1]);

}

//-----------------------------------
//  6:   Runge–Kutta–Fehlberg 4th order method  (v=1)
void ODE_RKF4(float *y, float *x, float h)
{


    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h * 2.0/9.0;
    xAux[1] = x[1] + k1[1] * h * 2.0/9.0;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + (k1[0] /12.0 + k2[0] /4.0) * h;
    xAux[1] = x[1] + (k1[1] /12.0 + k2[1] /4.0) /27.0;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + (k1[0] * 69.0 - k2[0] * 243.0 + k3[0] * 270.0)/128.0 * h;
    xAux[1] = x[1] + (k1[1] * 69.0 - k2[1] * 243.0 + k3[1] * 270.0)/128.0 * h;
    SystemODE(k4, xAux);

    xAux[0] = x[0] - (k1[0] * 17.0/12.0 - k2[0] * 27.0/4.0 + k3[0] * 27.0/5.0 - k4[0] * 16.0/15.0) * h;
    xAux[1] = x[1] - (k1[1] * 17.0/12.0 - k2[1] * 27.0/4.0 + k3[1] * 27.0/5.0 - k4[1] * 16.0/15.0) * h;
    SystemODE(k5, xAux);

    xAux[0] = x[0] + (k1[0] * 65.0/432.0 - k2[0] * 5.0/16.0 + k3[0] * 13.0/16.0 + k4[0] * 4.0/27.0 + k5[0] * 5.0/144.0) * h;
    xAux[1] = x[1] + (k1[1] * 65.0/432.0 - k2[1] * 5.0/16.0 + k3[1] * 13.0/16.0 + k4[1] * 4.0/27.0 + k5[1] * 5.0/144.0) * h;
    SystemODE(k6, xAux);

    //y[0] = x[0] + h * (k1[0]/9.0 + 0.0*k2[0] + 9.0/20.0*k3[0] + 16.0/45.0*k4[0] + k5[0]/12.0);
    //y[1] = x[1] + h * (k1[1]/9.0 + 0.0*k2[1] + 9.0/20.0*k3[1] + 16.0/45.0*k4[1] + k5[1]/12.0);
    y[0] = x[0] + h * (47.0/450.0*k1[0] + 0.0*k2[0] + 12.0/25.0*k3[0] + 32.0/225.0*k4[0] + k5[0]/30.0 + 6.0/25.0*k5[0]);
    y[1] = x[1] + h * (47.0/450.0*k1[1] + 0.0*k2[1] + 12.0/25.0*k3[1] + 32.0/225.0*k4[1] + k5[1]/30.0 + 6.0/25.0*k5[1]);

    /*

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
    y[1] = x[1] + h * (25/216*k1[1] + 0*k2[1] + 1408/2565*k3[1] + 2197/4104*k4[1] - 1/5*k5[1] + 0*k6[1]);

    */
}

////////////////
//  7:   Runge–Kutta–Fehlberg 5th order method
void ODE_RKF5(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h/4;
    xAux[1] = x[1] + k1[1] * h/4;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + (k1[0] * 3.0/32.0 + k2[0] * 9.0/32.0) * h;
    xAux[1] = x[1] + (k1[1] * 3.0/32.0 + k2[1] * 9.0/32.0) * h;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + (k1[0] * 1932.0/2197.0 - k2[0] * 7200.0/2197.0 + k3[0] * 7296.0/2197.0) * h;
    xAux[1] = x[1] + (k1[1] * 1932.0/2197.0 - k2[1] * 7200.0/2197.0 + k3[1] * 7296.0/2197.0) * h;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + (k1[0] * 439.0/216.0 - k2[0] * 8.0 + k3[0] * 3680.0/513.0 - k4[0] * 845.0/4104.0) * h;
    xAux[1] = x[1] + (k1[1] * 439.0/216.0 - k2[1] * 8.0 + k3[1] * 3680.0/513.0 - k4[1] * 845.0/4104.0) * h;
    SystemODE(k5, xAux);

    xAux[0] = x[0] - (k1[0] * 8.0/27.0 + k2[0] * 2.0 - k3[0] * 3544.0/2565.0 + k4[0] * 1859.0/4104.0 - k5[0] * 11.0/40.0) * h;
    xAux[1] = x[1] - (k1[1] * 8.0/27.0 + k2[1] * 2.0 - k3[1] * 3544.0/2565.0 + k4[1] * 1859.0/4104.0 - k5[1] * 11.0/40.0) * h;
    SystemODE(k6, xAux);

    y[0] = x[0] + h * (16.0/135.0*k1[0] + 0*k2[0] + 6656.0/12825.0*k3[0] + 28561.0/56430.0*k4[0] - 9.0/50.0*k5[0] + 2.0/55.0*k6[0]);
    y[1] = x[1] + h * (16.0/135.0*k1[1] + 0*k2[1] + 6656.0/12825.0*k3[1] + 28561.0/56430.0*k4[1] - 9.0/50.0*k5[1] + 2.0/55.0*k6[1]);

}



//  8:  RungeKutta 6th Order
void ODE_RK6(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], k7[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h;
    xAux[1] = x[1] + k1[1] * h;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + (k1[0] * 3.0 + k2[0]) /8.0 * h;
    xAux[1] = x[1] + (k1[1] * 3.0 + k2[1]) /8.0 * h;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + (k1[0] * 8.0 + k2[0] * 2.0 + k3[0] * 8.0) /27.0 * h;
    xAux[1] = x[1] + (k1[1] * 8.0 + k2[1] * 2.0 + k3[1] * 8.0) /27.0 * h;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + (k1[0] * 3.0 *(3.0 * sqrt(21.0) - 7.0) - k2[0] * 8.0 * (7.0 - sqrt(21.0)) + k3[0] * 48.0 * (7 - sqrt(21.0)) - k4[0] * 3.0 * (21.0 - sqrt(21.0))) /392.0 * h;
    xAux[1] = x[1] + (k1[1] * 3.0 *(3.0 * sqrt(21.0) - 7.0) - k2[1] * 8.0 * (7.0 - sqrt(21.0)) + k3[1] * 48.0 * (7 - sqrt(21.0)) - k4[1] * 3.0 * (21.0 - sqrt(21.0))) /392.0 * h;
    SystemODE(k5, xAux);

    xAux[0] = x[0] - (k1[0] * 5.0 *(231.0 + 51 * sqrt(21.0)) + k2[0] * 40.0 * (7.0 + sqrt(21.0)) + k3[0] * 320.0 * sqrt(21.0) - k4[0] * 3.0 * (21.0 + 121.0 * sqrt(21.0)) - k5[0] * 392.0 * (6.0 + sqrt(21.0))) /1960.0 * h;
    xAux[1] = x[1] - (k1[1] * 5.0 *(231.0 + 51 * sqrt(21.0)) + k2[1] * 40.0 * (7.0 + sqrt(21.0)) + k3[1] * 320.0 * sqrt(21.0) - k4[1] * 3.0 * (21.0 + 121.0 * sqrt(21.0)) - k5[1] * 392.0 * (6.0 + sqrt(21.0))) /1960.0 * h;
    SystemODE(k6, xAux);

    xAux[0] = x[0] + (k1[0] * 15.0 *(22.0 + sqrt(21.0) * 7.0) + k2[0] * 120.0 + k3[0] * 40.0 * (7 * sqrt(21.0) - 5.0) - k4[0] * 63.0 * (3.0 * sqrt(21.0) - 2.0) - k5[0] * 14.0 * (49.0 + 9.0 * sqrt(21.0)) + k6[0] * 70.0 * (7.0 - sqrt(21.0))) /180.0 * h;
    xAux[1] = x[1] + (k1[1] * 15.0 *(22.0 + sqrt(21.0) * 7.0) + k2[1] * 120.0 + k3[1] * 40.0 * (7 * sqrt(21.0) - 5.0) - k4[1] * 63.0 * (3.0 * sqrt(21.0) - 2.0) - k5[1] * 14.0 * (49.0 + 9.0 * sqrt(21.0)) + k6[1] * 70.0 * (7.0 - sqrt(21.0))) /180.0 * h;
    SystemODE(k7, xAux);

    y[0] = x[0] + h/180.0 * (9.0*k1[0] + 0.0*k2[0] + 64.0*k3[0] + 0.0*k4[0] + 49.0*k5[0] + 49.0*k6[0] + 9.0*k7[0]);
    y[1] = x[1] + h/180.0 * (9.0*k1[1] + 0.0*k2[1] + 64.0*k3[1] + 0.0*k4[1] + 49.0*k5[1] + 49.0*k6[1] + 9.0*k7[1]);


}


//  9:  Runge–Kutta–Fehlberg 7th order method
void ODE_RKF7(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], k7[2], k8[2], k9[2], k10[2], k11[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h * 2.0/27.0;
    xAux[1] = x[1] + k1[1] * h * 2.0/27.0;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + (k1[0] + 3.0 * k2[0]) /36.0 * h;
    xAux[1] = x[1] + (k1[1] + 3.0 * k2[1]) /36.0 * h;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + (k1[0] - k2[0] * 0.0 + k3[0] * 3.0) * h /24.0;
    xAux[1] = x[1] + (k1[1] - k2[1] * 0.0 + k3[1] * 3.0) * h /24.0;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + (k1[0] * 20.0 + k2[0] * 0.0 - k3[0] * 75.0 + k4[0] * 75.0) /48.0 * h;
    xAux[1] = x[1] + (k1[1] * 20.0 + k2[1] * 0.0 - k3[1] * 75.0 + k4[1] * 75.0) /48.0 * h;
    SystemODE(k5, xAux);

    xAux[0] = x[0] + (k1[0] - k2[0] * 0.0 + k3[0] * 0.0 - k4[0] * 5.0 + k5[0] * 4.0) /20.0 * h;
    xAux[1] = x[1] + (k1[1] - k2[1] * 0.0 + k3[1] * 0.0 - k4[1] * 5.0 + k5[1] * 4.0) /20.0 * h;
    SystemODE(k6, xAux);

    xAux[0] = x[0] - (k1[0] * 25.0 + k2[0] * 0 - k3[0] * 0.0 - k4[0] * 125.0 + k5[0] * 260.0) /108.0 * h;
    xAux[1] = x[1] - (k1[1] * 25.0 + k2[1] * 0 - k3[1] * 0.0 - k4[1] * 125.0 + k5[1] * 260.0) /108.0 * h;
    SystemODE(k7, xAux);

    xAux[0] = x[0] + (k1[0] * 31.0/300.0 + k2[0] * 0.0 + k3[0] * 0.0 - k4[0] * 0.0 + k5[0] * 61.0/225.0 - k6[0] * 2.0/9.0 + k7[0] * 13.0/900.0) * h;
    xAux[1] = x[1] + (k1[1] * 31.0/300.0 + k2[1] * 0.0 + k3[1] * 0.0 - k4[1] * 0.0 + k5[1] * 61.0/225.0 - k6[1] * 2.0/9.0 + k7[1] * 13.0/900.0) * h;
    SystemODE(k8, xAux);

    xAux[0] = x[0] + (k1[0] * 2.0 - k2[0] * 0.0 + k3[0] * 0.0 - k4[0] * 53.0/6.0 + k5[0] * 750.0/45.0 - k6[0] * 107.0/9.0 + k7[0] * 67.0/90.0 + k8[0] * 3.0) * h;
    xAux[1] = x[1] + (k1[1] * 2.0 - k2[1] * 0.0 + k3[1] * 0.0 - k4[1] * 53.0/6.0 + k5[1] * 750.0/45.0 - k6[1] * 107.0/9.0 + k7[1] * 67.0/90.0 + k8[1] * 3.0) * h;
    SystemODE(k9, xAux);

    xAux[0] = x[0] - (k1[0] * 91.0/108.0 + k2[0] * 0.0 + k3[0] * 0.0 - k4[0] * 23.0/108.0 + k5[0] * 976.0/135.0 - k6[0] * 311.0/54.0 + k7[0] * 19.0/60.0 - k8[0] * 17.0/6.0 + k9[0] /12.0) * h;
    xAux[1] = x[1] - (k1[1] * 91.0/108.0 + k2[1] * 0.0 + k3[1] * 0.0 - k4[1] * 23.0/108.0 + k5[1] * 976.0/135.0 - k6[1] * 311.0/54.0 + k7[1] * 19.0/60.0 - k8[1] * 17.0/6.0 + k9[1] /12.0) * h;
    SystemODE(k10, xAux);

    xAux[0] = x[0] + (k1[0] * 2383.0/4100.0 + k2[0] * 0.0 + k3[0] * 0.0 - k4[0] * 341.0/164.0 + k5[0] * 4496.0/1025.0 - k6[0] * 301.0/82.0 + k7[0] * 2133.0/4100.0 + k8[0] * 45.0/82.0 + k9[0] * 45.0/164.0 + k10[0] * 18.0/41.0) * h;
    xAux[1] = x[1] + (k1[1] * 2383.0/4100.0 + k2[1] * 0.0 + k3[1] * 0.0 - k4[1] * 341.0/164.0 + k5[1] * 4496.0/1025.0 - k6[1] * 301.0/82.0 + k7[1] * 2133.0/4100.0 + k8[1] * 45.0/82.0 + k9[1] * 45.0/164.0 + k10[0] * 18.0/41.0) * h;
    SystemODE(k11, xAux);

    y[0] = x[0] + h * (41.0/840.0 * k1[0] + 0.0* k2[0] + 0.0*k3[0] + 0.0*k4[0] + 0.0*k5[0] + 34.0/105.0*k6[0] + 9.0/35.0*k7[0] + 9.0/35.0*k8[0] + 9.0/280.0*k9[0] + 9.0/280.0*k10[0] + 41.0/840.0*k11[0]);
    y[1] = x[1] + h * (41.0/840.0 * k1[1] + 0.0* k2[1] + 0.0*k3[1] + 0.0*k4[1] + 0.0*k5[1] + 34.0/105.0*k6[1] + 9.0/35.0*k7[1] + 9.0/35.0*k8[1] + 9.0/280.0*k9[1] + 9.0/280.0*k10[1] + 41.0/840.0*k11[1]);

}

//  10:  Huta's Improved RungeKutta 6th Order
void ODE_HIRK6(float *y, float *x, float h)
{

    float k1[2], k2[2], k3[2], k4[2], k5[2], k6[2], k7[2], k8[2], xAux[2];
    SystemODE(k1, x);

    xAux[0] = x[0] + k1[0] * h /9.0;
    xAux[1] = x[1] + k1[1] * h /9.0;
    SystemODE(k2, xAux);

    xAux[0] = x[0] + (k1[0] * 3.0 + k2[0]) /24.0 * h;
    xAux[1] = x[1] + (k1[1] * 3.0 + k2[1]) /24.0 * h;
    SystemODE(k3, xAux);

    xAux[0] = x[0] + (k1[0] - k2[0] * 3.0 + k3[0] * 4.0) /6.0 * h;
    xAux[1] = x[1] + (k1[1] - k2[1] * 3.0 + k3[1] * 4.0) /6.0 * h;
    SystemODE(k4, xAux);

    xAux[0] = x[0] + (k1[0]  - k2[0] * 0.0 + k3[0] * 0.0 + k4[0] * 3.0) /8.0 * h;
    xAux[1] = x[1] + (k1[1]  - k2[1] * 0.0 + k3[1] * 0.0 + k4[1] * 3.0) /8.0 * h;
    SystemODE(k5, xAux);

    xAux[0] = x[0] - (k1[0] * 4.0 + k2[0] * 21.0 - k3[0] * 46.0 + k4[0] * 29.0  - k5[0] * 10.0) /3.0 * h;
    xAux[1] = x[1] - (k1[1] * 4.0 + k2[1] * 21.0 - k3[1] * 46.0 + k4[1] * 29.0  - k5[1] * 10.0) /3.0 * h;
    SystemODE(k6, xAux);

    xAux[0] = x[0] - (k1[0] * 8.0 - k2[0] * 99.0 + k3[0] * 84.0 + k4[0] * 0.0 - k5[0] * 44.0 - k6[0] * 9.0) /72.0 * h;
    xAux[1] = x[1] - (k1[1] * 8.0 - k2[1] * 99.0 + k3[1] * 84.0 + k4[1] * 0.0 - k5[1] * 44.0 - k6[1] * 9.0) /72.0 * h;
    SystemODE(k7, xAux);

    xAux[0] = x[0] + (k1[0] * 107.0 - k2[0] * 243.0 + k3[0] * 0.0 + k4[0] * 354.0 - k5[0] * 172.0 - k6[0] * 36.0 + k6[0] * 72.0) /82.0 * h;
    xAux[1] = x[1] + (k1[1] * 107.0 - k2[1] * 243.0 + k3[1] * 0.0 + k4[1] * 354.0 - k5[1] * 172.0 - k6[1] * 36.0 + k6[1] * 72.0) /82.0 * h;
    SystemODE(k8, xAux);

    y[0] = x[0] + h/840.0 * (41.0*(k1[0] + k8[0]) + 216.0*(k3[0] + k7[0]) + 27.0*(k4[0] + k6[0]) + 272.0*k5[0] + 0.0*k2[0]);
    y[1] = x[1] + h/840.0 * (41.0*(k1[1] + k8[1]) + 216.0*(k3[1] + k7[1]) + 27.0*(k4[1] + k6[1]) + 272.0*k5[1] + 0.0*k2[1]);

}


//==============================================================


