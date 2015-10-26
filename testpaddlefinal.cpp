#include <iostream>
#include <stdio.h>
#include <cmath>
using namespace std;
// ###############################################################
    // FlexPaddle Functions
    //      total length TotalLength
    //      Y where to apply the motion law : a
    //      motion law : S0
    //      y: ordinate where to calcultate paddle pos
// ###############################################################

     double ShapeBeforeAppPoint(double y,double S0,double a){
        return  (-S0/(2. * pow(a,3))*(pow(y,3) - 3.*a*pow(y,2)));
     }

     double ShapeAfterAppPoint(double y,double S0,double a){
         return  (-S0/(2. * pow(a,3))*(pow(a,3) - 3.*y*pow(a,2)));
     }

     double PaddleFlexiShape(double y,double S0,double a, double TotalLength){
         if ((y<=a) & (y>=0.)){
         double res=ShapeBeforeAppPoint( y,S0, a);
           return res;
         }
         if ((y>a) & (y<=TotalLength)){
           return ShapeAfterAppPoint( y,S0, a);
         }
         else{
           return 0. ;
         }
     }
     double ShapeBeforeAppPointNormalCosine(double y,double S0,double a){
        double dy= (-S0/(2. * pow(a,3))*(3.*pow(y,2) - 2.*3.*a*y));
	double norm=sqrt(1+pow(dy,2.));
	double cosine=1./norm;
	return cosine;
     }
     double ShapeBeforeAppPointNormalSine(double y,double S0,double a){
        double dy= (-S0/(2. * pow(a,3))*(3.*pow(y,2) - 2.*3.*a*y));
	double norm=sqrt(1+pow(dy,2.));
	double sine=dy/norm;
	return sine;
     }
     double ShapeAfterAppPointNormalCosine(double y,double S0,double a){
        double dy = (-S0/(2. * pow(a,3))*( - 3.*pow(a,2)));
	double norm=sqrt(1+pow(dy,2.));
	double cosine=1./norm;
	return cosine;
     }
     double ShapeAfterAppPointNormalSine(double y,double S0,double a){
        double dy = (-S0/(2. * pow(a,3))*( - 3.*pow(a,2)));
	double norm=sqrt(1+pow(dy,2.));
	double sine=dy/norm;
	return sine;
     }
     double PaddleFlexiShapeNormalCosine(double y,double S0,double a, double TotalLength){
         if ((y<=a) & (y>=0.)){
         double res=ShapeBeforeAppPointNormalCosine( y,S0, a);
           return res;
         }
         if ((y>a) & (y<=TotalLength)){
           return ShapeAfterAppPointNormalCosine( y,S0, a);
         }
         else{
           return 1. ;
         }
     }
     double PaddleFlexiShapeNormalSine(double y,double S0,double a, double TotalLength){
         if ((y<=a) & (y>=0.)){
         double res=ShapeBeforeAppPointNormalSine( y,S0, a);
           return res;
         }
         if ((y>a) & (y<=TotalLength)){
           return ShapeAfterAppPointNormalSine( y,S0, a);
         }
         else{
           return 0. ;
         }
     }
     
// ###############################################################
    // Strait Paddle Functions
    //      total length TotalLength
    //      Y where to apply the motion law : a
    //      motion law : S0
    //      y: ordinate where to calcultate paddle pos
// ###############################################################

     double PaddleStraitShape(double y,double S0,double a, double TotalLength){
         if ((y>=0.) & (y<=TotalLength)){
           return ( y * (S0/a));
         }
         else{
           return 0. ;
         }
      }
     double PaddleStraitShapeNormalCosine(double y,double S0,double a, double TotalLength){
         if ((y>=0.) & (y<=TotalLength)){
           return ( 1./sqrt(1.+ (S0/a)*(S0/a)));
         }
         else{
           return 1. ;
         }
      }
     double PaddleStraitShapeNormalSine(double y,double S0,double a, double TotalLength){
         if ((y>=0.) & (y<=TotalLength)){
           return ( (S0/a)/sqrt(1.+ (S0/a)*(S0/a)));
         }
         else{
           return 0. ;
         }
      }
// ###############################################################
    // Piston Paddle Functions
    //      total length TotalLength
    //      motion law : S0
    //      y: ordinate where to calcultate paddle pos
// ###############################################################

     double PaddlePiston(double y,double S0, double TotalLength){
         if ((y>=0.) & (y<=TotalLength)){
           return (S0);
         }
         else{
           return 0. ;
         }
      }
//###############################################################
    // 3D Paddle Phase
//###############################################################
     double Phase3D(double Xconv,double Zconv,double Omega,double ZZ,double gravity){
        if (Xconv>0.){
          double Kp=(pow(Omega,2)/gravity);
          double theta=atan2(ZZ-Zconv,Xconv);
          double PPhase=Kp*(Xconv*cos(theta)+Zconv*sin(theta))-Kp*sin(theta)*ZZ+0.5*M_PI;
          return PPhase;
        }
        else {
          return 0.;
        }
     }

// ###############################################################
    // Paddle motion laws
// ###############################################################
     double Class3PaddleOS(double Omega,double t,double coeft2,double Phase){
        double CCC = (Omega *(t- 0.5*Omega *coeft2*pow(t,2))+Phase) ;
        double OS = sin(CCC) ;
        return OS ;
     }

     double Class3PaddleOST(double Omega,double t,double coeft2,double Phase){
        double CCC = (Omega *(t - 0.5*Omega *coeft2*pow(t,2))+Phase) ;
        double OST = Omega *(1. - Omega *coeft2*t)*cos(CCC) ;
        return OST ;
     }

     double Class3PaddleDAMP(double Omega,double t,double N){
        double AAA= (4.*(-2.*N*M_PI + Omega*t))/(N *M_PI);
        double BBB= (4.*Omega*t)/(N*M_PI);
        double DAMP=(1. + tanh(BBB))*(1. - tanh(AAA));
        return DAMP;
     }

     double Class3PaddleDAMPT(double Omega,double t,double N){
        double AAA= (4.*(-2.*N*M_PI + Omega*t))/(N *M_PI);
        double BBB= (4.*Omega*t)/(N*M_PI);
        double DDD= (4.*Omega) / (N*M_PI);

        double CAAA=1./cosh(AAA);
        double CBBB=1./cosh(BBB);

        double DAMPT=DDD*(-(CAAA*CAAA)*(1. + tanh(BBB)) + (CBBB*CBBB)*(1 - tanh(AAA)));

        return DAMPT;
     }

     double Class3MotionLaw(double Omega,double t,double coeft2,double N,double Xconv,double Zconv,double ZZ,double gravity){
           return Class3PaddleOS(Omega,t,coeft2,Phase3D(Xconv,Zconv,Omega,ZZ,gravity))*Class3PaddleDAMP(Omega,t,N) ;
     }

     double Class3MotionLawVelocity(double Omega,double t,double coeft2,double N,double Xconv,double Zconv,double ZZ,double gravity){
           return Class3PaddleOST(Omega,t,coeft2,Phase3D(Xconv,Zconv,Omega,ZZ,gravity))*Class3PaddleDAMP(Omega,t,N) \
           + Class3PaddleOS(Omega,t,coeft2,Phase3D(Xconv,Zconv,Omega,ZZ,gravity))*Class3PaddleDAMPT(Omega,t,N);
     }

     double Class3Paddle(double YY,double ZZ,double t,double Omega,double coeft2,double N,double Xconv,double Zconv,double gravity,double a, double TotalLength){
           double S0=Class3MotionLaw(Omega, t, coeft2, N, Xconv, Zconv, ZZ, gravity);
           return PaddleStraitShape(YY, S0, a, TotalLength);
           //return PaddleFlexiShape (YY, S0, a, TotalLength);
           //return PaddlePiston(YY, S0, TotalLength);
     }

     double Class3PaddleVelocity(double YY,double ZZ,double t,double Omega,double coeft2,double N,double Xconv,double Zconv,double gravity,double a, double TotalLength){
           double S0=Class3MotionLawVelocity(Omega, t, coeft2, N, Xconv, Zconv, ZZ, gravity);
           return PaddleStraitShape(YY, S0, a, TotalLength);
           //return PaddleFlexiShape (YY, S0, a, TotalLength);
           //return PaddlePiston(YY, S0, TotalLength);
     }

     double Class3PaddleVelocityU(double YY,double ZZ,double t,double Omega,double coeft2,double N,double Xconv,double Zconv,double gravity,double a, double TotalLength){
           double S0=Class3MotionLawVelocity(Omega, t, coeft2, N, Xconv, Zconv, ZZ, gravity);
           return PaddleStraitShape(YY, S0, a, TotalLength)*PaddleStraitShapeNormalCosine(YY, S0, a, TotalLength);
           //return PaddleFlexiShape (YY, S0, a, TotalLength);
           //return PaddlePiston(YY, S0, TotalLength);
     }

     double Class3PaddleVelocityV(double YY,double ZZ,double t,double Omega,double coeft2,double N,double Xconv,double Zconv,double gravity,double a, double TotalLength){
           double S0=Class3MotionLawVelocity(Omega, t, coeft2, N, Xconv, Zconv, ZZ, gravity);
           return PaddleStraitShape(YY, S0, a, TotalLength)*PaddleStraitShapeNormalSine(YY, S0, a, TotalLength);
           //return PaddleFlexiShape (YY, S0, a, TotalLength);
           //return PaddlePiston(YY, S0, TotalLength);
     }
int main() {
	// your code goes here
	double d=1.2;
	printf("%f \n",d);
	
	double S0 = 1;
	double a = 0.5;
	double TotalLength=1.;
	double Pcoeft2=(0.018/1.78);
        double PN=5;
        double PXconv=-1.;
        double PZconv=0.;
        double paddletimeshift = -6.;
	double omega=(2.* M_PI);
	double t=0.;
	double g=2.* M_PI;
	double Papp=0.5;
	double PTotalLength=1.2*Papp;

	printf("%f \n",Class3PaddleVelocityU(0.,0., t+paddletimeshift,omega,Pcoeft2,PN,PXconv,PZconv,g,Papp, PTotalLength));
	printf("%f \n",Class3PaddleVelocityU(0.2,0., t+paddletimeshift,omega,Pcoeft2,PN,PXconv,PZconv,g,Papp, PTotalLength));
	printf("%f \n",Class3PaddleVelocityU(0.4,0., t+paddletimeshift,omega,Pcoeft2,PN,PXconv,PZconv,g,Papp, PTotalLength));
	printf("%f \n",Class3PaddleVelocityU(0.5,0., t+paddletimeshift,omega,Pcoeft2,PN,PXconv,PZconv,g,Papp, PTotalLength));
	printf("%f \n",Class3PaddleVelocityU(0.6,0., t+paddletimeshift,omega,Pcoeft2,PN,PXconv,PZconv,g,Papp, PTotalLength));
	printf("%f \n",Class3PaddleVelocityU(0.7,0., t+paddletimeshift,omega,Pcoeft2,PN,PXconv,PZconv,g,Papp, PTotalLength));
	printf("%f \n",Class3PaddleVelocityU(0.9,0., t+paddletimeshift,omega,Pcoeft2,PN,PXconv,PZconv,g,Papp, PTotalLength));
	
	
	
	return 0;
}
