// Máquina de inducción control vectorial directo
// Rs=0.435 Ls=0.0713 Lr=0.0713 Lm0=0.0693 Lls=0.002 Llr=0.002 Rr=0.816
// Voltaje línea-línea 220 Vrms
// versión 1 24-Jun-2021
// Programado por José Restrepo - USB
// Para
// Máquina Kraus 3hp
/////////////////////////////////////////////////////////////////////
#include<math.h>
#include<stdlib.h>
#include<stdio.h>
// Constantes útiles

#define M_SQRT23		0.81649658092772603446
#define M_LOG2E         1.4426950408889634074
#define M_LOG10E        0.43429448190325182765
#define M_LN2           0.69314718055994530942
#define M_LN10          2.30258509299404568402
#define	M_2PI			6.2831853071795864769
#define M_PI            3.14159265358979323846
#define M_PI_2          1.57079632679489661923
#define M_PI_4          0.78539816339744830962
#define M_1_PI          0.31830988618379067154
#define M_2_PI          0.63661977236758134308
#define M_2PI_3			2.0943951023932
#define M_2_SQRTPI      1.12837916709551257390
#define M_SQRT2         1.41421356237309504880
#define M_SQRT2_2       0.70710678118654752440
#define	M_1_SQRT6		0.40824829046386301637
#define PI              M_PI
#define PI2             M_PI_2
#define M_SQRT3         1.73205080756887719317
#define M_SQRT3_2       0.86602540378443859658
#define M_SQRT6_2       1.22474487139158894066
#define CTE3			0.816497
#define CTE1			0.408248
#define CTE2			0.707107
#define fclipf(ns,m)     ((fabs(ns)>m)?(copysign (m,ns)):ns)
// Parametros para adquisición

#define LEN				15000
#define	STRIDE			1
#define	TEST			100000

//Frecuencia de muestreo

#define Fs      		10000
#define Fs_1			1e-4
#define	Ts_i			Fs_1


#define PWMMAX			800
//#define	CPWM_2			100
//#define	SLOPE			CPWM2/Ts_i
//#define	SLOPE_1			Ts_i/CPWM2
#define	Vdc		400.0

#define PWM

float kr,ks,k1,k2,k3,k4,k5,k6,k7;
float Vsx,Vsy;
float Te,Tl=0.0,J=0.089,P=2;

void ode(float *yout, float *x0,float paso);
void model(float *X, float *Y);

int main(void)
{
 float x[]={0,0,0,0,0},y[]={0,0,0,0,0};

 // Parámetros de la Máquina

 float Rs=0.435,Ls=0.0713,Lr=0.0713,Lm0=0.0693,Rr=0.816,Lm;

 // Variables eléctricas

 int it;
 float MIs,MFlr;

 //float angf,dangf,dang,Vp,VP,Km;
 //float Flsx,Flsy,Tx,Fls;
 //float Va,Vb,Vc;
 float Isa,Isb,Isc,wr;

 int A,B,C,flagst=1;
 int Da,Db,Dc,Dmax,Dmin,Dx,DT;
 float t1,t2,t3,t4,t5,t6,t_a,SLOPE;
 float theta_e;
 float err_wr,wr_ref,prop,Kps,xin,Kis,Isq,Isd;
 float vai,vbi,vci,Err_a,Err_b,Err_c,Kii,Kpi,Vsa,Vsb,Vsc;
 float Isx,Isy,Isa_ref,Isb_ref,Isc_ref;
 float Tr,Imr_ref,Imr[2],Ks,Kr,Ke,wsl=0;

 FILE *ptr;


 ptr=fopen("Imr.dat","w+");
 SLOPE=Ts_i/(2.0*PWMMAX);

 if(ptr==NULL)
 {
     printf("No se pudo crear el archivo de datos.\n");
     exit(-1);
 }

 Lm=Lm0;
 Imr_ref=8.0;
 Isd=8.0;

 Vsx=0;
 Vsy=0;
 Te=0;

 ////////////////////////////////////////////////////
 // Modelo de la máquina real
 Lm=Lm0;
 ks=1.0/(Ls*Lr-Lm*Lm);
 kr=1.0/Lr;
 k1=Lr*ks;
 Tr=Lr/Rr;

 k2=(Lr*Lr*Rs+Lm*Lm*Rr)*ks*kr;
 k4=Lm*ks;
 k6=Rr*kr;
 k5=k6*Lm;
 k3=k5*ks;
 k7=Lm/Lr;

 Ks=1/Ts_i;
 Kr=1/Tr;
 Ke=Ks+Kr;
 ////////////////////////////////////////////////////

 for(it=0;it<5;it++)
   x[it]=y[it]=0;

 // El voltaje línea-línea es 220 Vrms => Vap=220*sqrt(2)/sqrt(3)
 //VP=Vll*(2.0/3.0); // Vsx=sqrt(2/3)*(va+vb.a+vc.a^2)=sqrt(2/3)*(1.5*va)
 //angf=0;
 //dangf=M_2PI*Ts_i*fe;//dangf=M_PI*0.012;

 //fw=5;
 Kps=2.5;
 Kis=0.002;

 Kii=0.005;
 Kpi=0.05;
 wr_ref=50; // Cambiar
 for(it=0;it<LEN*STRIDE;it++)
  {

   Imr[1]=Imr[0];
   Imr[0]=(Imr[1]*Ks+Isd*Kr)/Ke;

  	// Control PI Lazo de velocidad

  	err_wr=(wr_ref - wr);
  	prop=Kps*err_wr;
  	xin=fclipf((xin+Kis*err_wr),10.0);

  	Isq=fclipf((prop+xin),16.0);

  	if((Imr[0]<0.98*Imr_ref)&&(flagst==1))
  	 {
  	  Isq=0;
  	  Isd=16;
  	  flagst=1;
  	 }
  	else
  	 {
  	  flagst=0;
  	  Isd=Imr_ref;
  	 }
   	//  Only for non ideal



   	if(Imr[0]<=0) wsl=0;
   	else wsl=Isq/(Tr*Imr[0]);

   	theta_e+=P*(wr+wsl)*Ts_i;

   	//theta_e=atan2f(x[3],x[2]);

   	if(theta_e>=M_PI) theta_e-=M_2PI;
   	if(theta_e<=-M_PI) theta_e+=M_2PI;

   	Isx=Isd*cos(theta_e)-Isq*sin(theta_e);

   	Isy=Isd*sin(theta_e)+Isq*cos(theta_e);

   	Isa_ref=M_SQRT23*Isx;
   	Isb_ref=M_SQRT2_2*Isy-M_1_SQRT6*Isx;
   	Isc_ref=-(Isa_ref+Isb_ref);

   	Isa=M_SQRT23*x[0];
   	Isb=M_SQRT2_2*x[1]-M_1_SQRT6*x[0];
   	Isc=-(Isa+Isb);

   // Controladores por fase

   /*if(Isa>Isa_ref)  Va=-0.5;
   else Va=0.5;

   if(Isb>Isb_ref)  Vb=-0.5;
   else Vb=0.5;

   if(Isc>Isc_ref)  Vc=-0.5;
   else Vc=0.5;

   Da=(PWMMAX*(0.5+Va));
   if(Da>1) Da=1;
   if(Da<0) Da=0;
   if(Db>1) Db=1;
   if(Db<0) Db=0;
   if(Dc>1) Dc=1;
   if(Dc<0) Dc=0;
   Db=(PWMMAX*(0.5+Vb));
   Dc=(PWMMAX*(0.5+Vc));*/
    Err_a=Isa_ref-Isa;

   	vai=fclipf((vai+Kii*Err_a),0.3);
   	Vsa=fclipf((vai+Kpi*Err_a),0.5);

   	Err_b=Isb_ref-Isb;

   	vbi=fclipf((vbi+Kii*Err_b),0.3);
   	Vsb=fclipf((vbi+Kpi*Err_b),0.5);

   	Err_c=Isc_ref-Isc;

   	vci=fclipf((vci+Kii*Err_c),0.3);
   	Vsc=fclipf((vci+Kpi*Err_c),0.5);

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
   Da=(PWMMAX*(0.5+Vsa));
   Db=(PWMMAX*(0.5+Vsb));
   Dc=(PWMMAX*(0.5+Vsc));
   // Limitar Da, Db y Dc en el rango [0,1]
   DT=Da+Db+Dc;
   //Dmax=max(max(Da,Db),Dc);
   Dmax=((Da>Db)?Da:Db);
   Dmax=(Dmax>Dc)?Dmax:Dc;
   //Dmin=min(min(Da,Db),Dc);
   Dmin=(Da<Db)?Da:Db;
   Dmin=(Dmin<Dc)?Dmin:Dc;
   Dx=DT-Dmax-Dmin;

   t1=Dmin*SLOPE;
   t2=Dx*SLOPE;
   t3=Dmax*SLOPE;
   t4=Ts_i-t3;
   t5=Ts_i-t2;
   t6=Ts_i-t1;
   t_a=0.0;
     	// Análisis desde 0->t1

   if(t1!=0)
   //if(Dmin != 0)
    {
     A=B=C=1;
     Vsx=0;	//Vdc*(A-0.5*B-0.5*C);
     Vsy=0;	//Vdc*M_SQRT3_2*(B-C);
     ode(y,x,t1);
     x[0]=y[0];
     x[1]=y[1];
     x[2]=y[2];
     x[3]=y[3];
     x[4]=y[4];
     t_a=t1;
    }

       // Análisis de t1->t2

   if(t1!=t2)
   //if(Dx != Dmin)
    {
     A=B=C=1;
     if(Da<=Dmin) A=0;
     if(Db<=Dmin) B=0;
     if(Dc<=Dmin) C=0;
     Vsx=Vdc*M_SQRT23*(A-0.5*B-0.5*C);
     Vsy=Vdc*M_SQRT2_2*(B-C);
     ode(y,x,(t2-t_a));
     x[0]=y[0];
     x[1]=y[1];
     x[2]=y[2];
     x[3]=y[3];
     x[4]=y[4];
     t_a=t2;
    }
   		// Análisis de t2->t3

   if(t2!=t3)
    {
     A=B=C=1;
     if(Da<=Dx) A=0;
     if(Db<=Dx) B=0;
     if(Dc<=Dx) C=0;
     Vsx=Vdc*M_SQRT23*(A-0.5*B-0.5*C);
     Vsy=Vdc*M_SQRT2_2*(B-C);
     ode(y,x,(t3-t_a));
     x[0]=y[0];
     x[1]=y[1];
     x[2]=y[2];
     x[3]=y[3];
     x[4]=y[4];
     t_a=t3;
    }

   // Análisis de t3->t4

   if(t3!=t4)
    {
     A=B=C=0;
     //tx=t4*SLOPE_1;
     Vsx=0;	//Vdc*(A-0.5*B-0.5*C);
     Vsy=0;	//Vdc*M_SQRT3_2*(B-C);
     ode(y,x,(t4-t_a));
     x[0]=y[0];
     x[1]=y[1];
     x[2]=y[2];
     x[3]=y[3];
     x[4]=y[4];
     t_a=t4;
    }

     // Análisis de t4->t5

   if(t4!=t5)
    {
     A=B=C=1;
     if(Da<Dmax) A=0;
     if(Db<Dmax) B=0;
     if(Dc<Dmax) C=0;
     Vsx=Vdc*M_SQRT23*(A-0.5*B-0.5*C);
     Vsy=Vdc*M_SQRT2_2*(B-C);
     ode(y,x,(t5-t_a));
     x[0]=y[0];
     x[1]=y[1];
     x[2]=y[2];
     x[3]=y[3];
     x[4]=y[4];
     t_a=t5;
    }

   // Análisis de t5->t6

   if(t5!=t6)
    {
     A=B=C=1;
     //V_pwm=t1;
     //tx=t6*SLOPE_1;
     if(Da<Dx) A=0;
     if(Db<Dx) B=0;
     if(Dc<Dx) C=0;
     Vsx=Vdc*M_SQRT23*(A-0.5*B-0.5*C);
     Vsy=Vdc*M_SQRT2_2*(B-C);
     ode(y,x,(t6-t_a));
     x[0]=y[0];
     x[1]=y[1];
     x[2]=y[2];
     x[3]=y[3];
     x[4]=y[4];
     t_a=t6;
    }

   // Análisis de t6->Ts_i

   if(t6!=Ts_i)
    {
     A=B=C=1;
     Vsx=0;	//Vdc*(A-0.5*B-0.5*C);
     Vsy=0;	//Vdc*M_SQRT3_2*(B-C);
     ode(y,x,Ts_i-t6);
     x[0]=y[0];
     x[1]=y[1];
     x[2]=y[2];
     x[3]=y[3];
     x[4]=y[4];
    }

   if(it%STRIDE==0)
    {
     Isa=x[0]*M_SQRT23;		//M_SQRT2_2       0.70710678118654752440
							//M_1_SQRT6		0.40824829046386301637
     Isb=-x[0]*M_1_SQRT6+x[1]*M_SQRT2_2;
     wr=x[4];
     MIs=sqrt(x[0]*x[0]+x[1]*x[1]);
     MFlr=sqrt(x[2]*x[2]+x[3]*x[3]);
     fprintf(ptr,"%e %e %e %e %e %e %e %e %e\n",Isa_ref,Isa,x[2],x[3],wr,Te,MIs,MFlr,x[0]);
    }

  }

 exit(0);
}

void model(float *X, float *Y)
{
 float Isx,Isy,Flrx,Flry,w;

 Isx=X[0];
 Isy=X[1];
 Flrx=X[2];
 Flry=X[3];
 w=P*X[4];

 Y[0]=k1*Vsx-k2*Isx+k3*Flrx+k4*w*Flry;
 Y[1]=k1*Vsy-k2*Isy+k3*Flry-k4*w*Flrx;
 Y[2]=k5*Isx-k6*Flrx-w*Flry;
 Y[3]=k5*Isy-k6*Flry+w*Flrx;
 Te=P*k7*(Isy*Flrx-Isx*Flry);
 Y[4]=(Te-Tl)/J;
}


 // ODE yout=ode(ypfun, t0, tfinal, y0, paso)

void ode(float *yout,float *x0,float paso)
// x0[0]=Isx, x0[1]=Isy, x0[2]=Flrx; x0[3]=Flry; x0[4]=wr
{
 int i;
 float h;
 float s1[5],s2[5],s3[5],s4[5],xt[5];

 h = paso;
   // Compute the slopes
   //   s1 = feval(ypfun, t, y); s1 = s1(:);
 model(x0,s1);
 for(i=0;i<5;i++)
    xt[i]=x0[i]+h*s1[i]/2.0;

   //  s2 = feval(ypfun, t+h, y+h*s1); s2 = s2(:);
 model(xt,s2);
   //  s3 = feval(ypfun, t+h/2, y+h*(s1+s2)/4); s3 = s3(:);
 for(i=0;i<5;i++)
    xt[i]=x0[i]+h*s2[i]/2.0;

 model(xt,s3);
 for(i=0;i<5;i++)
    xt[i]=x0[i]+h*s3[i];
 model(xt,s4);
 for(i=0;i<5;i++)
   yout[i]=x0[i]+h*(s1[i]+2.0*s2[i]+2.0*s3[i]+s4[i])/6.0;
}

void ode1(float *yout,float *x0,float paso)
// x0[0]=Isx, x0[1]=Isy, x0[2]=Flrx; x0[3]=Flry; x0[4]=wr
{
 int i;
 float h;
 float s1[5];
 h = paso;
   // Compute the slopes
   //   s1 = feval(ypfun, t, y); s1 = s1(:);
 model(x0,s1);
 for(i=0;i<5;i++)
   yout[i]=x0[i]+h*s1[i];
}
