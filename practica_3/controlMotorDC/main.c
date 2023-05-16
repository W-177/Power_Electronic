#include<math.h>
#include<stdlib.h>
#include<stdio.h>

#define Ra 2.581
#define La 28e-3
#define Km 1.8
#define J 0.02215
#define B 0
#define Vref 240.0

#define LEN 5000
void ode(float *,float,float *);
void odeRK(float *yout,float step,float *x0);
void dfxy(float *, float *);

float vin,suma,suma2;
float data[LEN];
float k1,k2;

void main( void )
{
 int i,j;
 float x[2],y[2],entrada ;
 FILE *ptr;

 ptr=fopen("datos.dat","w+");
 if(ptr==NULL)
 {
     printf("No se pudo crear el archivo de datos.\n");
     exit(-1);
 }
 suma=0;

 x[0]=93.396;
 x[1]=0;
 y[0]=y[1]=0;

 vin=0;
 for(i=0;i<LEN;i++)
  {
   vin=240;
   ode(y,100e-6,x);
   x[0]=y[0];
   x[1]=y[1];
   fprintf(ptr,"%e %e\n",x[0],x[1]);
  }
 exit(0);
}

// This function provides the derivatives
void dfxy(float *X, float *Y)
{
 // System variables

 Y[0]=-Ra*X[0]/La-Km*X[1]/La+vin/La;
 Y[1]=Km*X[0]/J-B*X[1]/J;
}

 // ODE yout=ode(ypfun, t0, tfinal, y0)
 // fixed step RK4
void ode(float *Y,float step,float *X)
{
    float der[2];
    dfxy(X,der);
    Y[0]=X[0]+der[0]*step;
    Y[1]=X[1]+der[1]*step;
}

void odeRK(float *yout,float step,float *x0)
// x0[0]=Isx, x0[1]=Isy
{
 float hmax,h;
 float s1[2],s2[2],s3[2],s4[2],xt[2];

   // Compute the slopes
   //   s1 = feval(ypfun, t, y); s1 = s1(:);
   // Deberķamos suplir la derivada inicial?
 dfxy(x0,s1);
 xt[0]=x0[0]+step*s1[0]/2.0;
 xt[1]=x0[1]+step*s1[1]/2.0;

   //  s2 = feval(ypfun, t+h, y+h*s1); s2 = s2(:);
 dfxy(xt,s2);
   //  s3 = feval(ypfun, t+h/2, y+h*(s1+s2)/4); s3 = s3(:);
 xt[0]=x0[0]+step*s2[0]/2.0;
 xt[1]=x0[1]+step*s2[1]/2.0;

 dfxy(xt,s3);
 xt[0]=x0[0]+step*s3[0];
 xt[1]=x0[1]+step*s3[1];
 dfxy(xt,s4);
 yout[0]=x0[0]+step*(s1[0]+2.0*s2[0]+2.0*s3[0]+s4[0])/6.0;
 yout[1]=x0[1]+step*(s1[1]+2.0*s2[1]+2.0*s3[1]+s4[1])/6.0;
}
