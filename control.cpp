// controlDLL.cpp : Defines the entry point for the DLL application.
//
#ifdef WIN32
#include "stdafx.h"
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved )
{
    return TRUE;
}
#include <stdio.h>

#else // WIN32
#include "servo.h"

#endif // WIN32

#include "param.h"
#include "control.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
#include <stdio.h>
using std::min;
using std::max;

// struct for spline parameters
struct cSplineParam {
	float tf_max;
	float t0;
	PrVector6 a0;
	PrVector6 a1;
	PrVector6 a2;
	PrVector6 a3;
};

// global spline parameters
cSplineParam spline;

bool inv_kin( const PrVector3& pos, const PrMatrix3& rot, int elbow,
              PrVector& qOut, GlobalVariables& gv );
void OpDynamics( const PrVector& fPrime, GlobalVariables& gv );

PrVector6 velocitySaturate(GlobalVariables& gv);
void generateSplineTrajectory(GlobalVariables& gv);
bool InJointLimits(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************
  
void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv)
{
   // This code runs on every servo loop, just before the control law
}


void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
	gv.tau = gv.G;
}

void initOpenControl(GlobalVariables& gv) 
{
}

void initNjholdControl(GlobalVariables& gv) 
{
	gv.qd = gv.q;
	gv.dqd.zero();
}

void initJholdControl(GlobalVariables& gv) 
{
	initNjholdControl(gv);
}

void initNjmoveControl(GlobalVariables& gv) 
{
	gv.dqd.zero();
}

void initJmoveControl(GlobalVariables& gv) 
{
	gv.dqd.zero();
}

void initNjgotoControl(GlobalVariables& gv) 
{
	if(!InJointLimits(gv)) {
		gv.qd = gv.q;
	}
} 

void initJgotoControl(GlobalVariables& gv) 
{
	if(!InJointLimits(gv)) {
		gv.qd = gv.q;
	}
}

void initNjtrackControl(GlobalVariables& gv) 
{
	generateSplineTrajectory(gv);
}

void initJtrackControl(GlobalVariables& gv) 
{
	generateSplineTrajectory(gv);
}

void initNxtrackControl(GlobalVariables& gv) 
{
	PrVector6 qOut;
	if(!inv_kin(gv.Td.translation(), gv.Td.rotation().matrix(), gv.elbow, qOut, gv)) {
		gv.qd = qOut;
	} else {
		gv.dq = gv.q;
	}
}

void initXtrackControl(GlobalVariables& gv) 
{
	PrVector6 qOut;
	if(!inv_kin(gv.Td.translation(), gv.Td.rotation().matrix(), gv.elbow, qOut, gv)) {
		gv.qd = qOut;
	} else {
		gv.dq = gv.q;
	}s
} 

void initNholdControl(GlobalVariables& gv) 
{
}

void initHoldControl(GlobalVariables& gv) 
{
}

void initNgotoControl(GlobalVariables& gv) 
{
} 

void initGotoControl(GlobalVariables& gv) 
{
} 

void initNtrackControl(GlobalVariables& gv) 
{
}

void initTrackControl(GlobalVariables& gv) 
{
} 

void initPfmoveControl(GlobalVariables& gv) 
{
} 

void initLineControl(GlobalVariables& gv) 
{
}

void initProj1Control(GlobalVariables& gv) 
{
}

void initProj2Control(GlobalVariables& gv) 
{
}

void initProj3Control(GlobalVariables& gv) 
{
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{
   gv.tau = gv.G;
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njholdControl(GlobalVariables& gv) 
{
	gv.tau = -gv.kp*(gv.q - gv.qd)-gv.kv*(gv.dq - gv.dqd);
}

void jholdControl(GlobalVariables& gv) 
{
	PrVector tau_prime = -gv.kp*(gv.q - gv.qd)-gv.kv*(gv.dq - gv.dqd);
	gv.tau = gv.A * tau_prime + gv.B + gv.G;
}

void njmoveControl(GlobalVariables& gv)
{
   gv.tau = -gv.kp*(gv.q - gv.qd)-gv.kv*(gv.dq - gv.dqd) + gv.G;
}

void jmoveControl(GlobalVariables& gv)
{
   jholdControl(gv);
}

void njgotoControl(GlobalVariables& gv) 
{
	gv.tau = velocitySaturate(gv) + gv.G;
}

void jgotoControl(GlobalVariables& gv) 
{
   gv.tau = gv.A * velocitySaturate(gv) + gv.B + gv.G;
}

void njtrackControl(GlobalVariables& gv) 
{
	float t = gv.curTime - spline.t0;
	if(t <= spline.tf_max) {
		gv.qd   = spline.a0 + spline.a2 * pow(t,2) + spline.a3 * pow(t,3);
		gv.dqd  = 2 * spline.a2 * t + 3 * spline.a3 * pow(t,2);
		gv.ddqd = 2 * spline.a2 + 6 * spline.a3 * t; 
		gv.tau  = -gv.kp*(gv.q - gv.qd) - gv.kv*(gv.dq - gv.dqd) + gv.ddqd;
	} else {
		gv.qd = gv.q;
		njholdControl(gv);
	}
	gv.tau += gv.G;
}

void jtrackControl(GlobalVariables& gv)
{
   float t = gv.curTime - spline.t0;
	if(t <= spline.tf_max) {
		gv.qd   = spline.a0 + spline.a2 * pow(t,2) + spline.a3 * pow(t,3);
		gv.dqd  = 2 * spline.a2 * t + 3 * spline.a3 * pow(t,2);
		gv.ddqd = 2 * spline.a2 + 6 * spline.a3 * t; 
		PrVector6 tau_prime;
		tau_prime = -gv.kp*(gv.q - gv.qd) - gv.kv*(gv.dq - gv.dqd) + gv.ddqd;
		gv.tau = gv.A * tau_prime + gv.B;
	} else {
		gv.qd = gv.q;
		njholdControl(gv);
	}
	gv.tau += gv.G;
}

void nxtrackControl(GlobalVariables& gv) 
{
	gv.tau = -gv.kp*(gv.q - gv.qd) - gv.kv*(gv.dq - gv.dqd) + gv.G;
}

void xtrackControl(GlobalVariables& gv) 
{
   PrVector6 tau_prime;
   tau_prime = -gv.kp*(gv.q - gv.qd) - gv.kv*(gv.dq - gv.dqd);
   gv.tau = gv.A * tau_prime + gv.B + gv.G;
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement nholdControl
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement holdControl
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement ngotoControl
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement gotoControl
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement ntrackControl
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement trackControl
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement pfmoveControl
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement lineControl
}

void proj1Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj1Control
}

void proj2Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj2Control
}

void proj3Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj3Control
}


// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
   // Replace this code with any debug information you'd like to get
   // when you type "pdebug" at the prompt.
   printf( "This sample code prints the torque and mass\n" );
   gv.tau.display( "tau" );
   gv.A.display( "A" );
}

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/

void DH_transform (const PrVector& DH, PrMatrix& DH_matrix)
{
	DH_matrix[0][0] = cos(DH[3])           ; DH_matrix[0][1] = -sin(DH[3])          ;
	DH_matrix[0][2] = 0                    ;  DH_matrix[0][3] = DH[1]               ;
	DH_matrix[1][0] = sin(DH[3])*cos(DH[0]); DH_matrix[1][1] = cos(DH[3])*cos(DH[0]);
	DH_matrix[1][2] = -sin(DH[0])          ;  DH_matrix[1][3] = -DH[2]*sin(DH[0])   ;
	DH_matrix[2][0] = sin(DH[3])*sin(DH[0]); DH_matrix[2][1] = cos(DH[3])*sin(DH[0]);
	DH_matrix[2][2] = cos(DH[0])           ;  DH_matrix[2][3] = DH[2]*cos(DH[0])    ;
	DH_matrix[3][0] = 0                    ; DH_matrix[3][1] = 0                    ;
	DH_matrix[3][2] = 0                    ;  DH_matrix[3][3] = 1                   ;
}

bool inv_kin( const PrVector3& pos, const PrMatrix3& rot, int elbow,
              PrVector& qOut, GlobalVariables& gv )
{
   double pi = 3.14159265;
   //Find the position of the Wrist
   PrVector3 Z6_0, Wrist0;
   rot.getColumn(2,Z6_0);
   Wrist0 = pos - L6*Z6_0;
   /////////////////////
   //Compute theta1/////
   /////////////////////
   // We have from the expression of T04 the coordinates of the Wrirst (x4,y4,z4)
   // These expressions can be simplified to isolate theta1 which gives
   //  L1 = y4c1 - x4s1 ====>  L1/n = suc1 - cus1 = s(u - 1)
   double l = L1/sqrt(Wrist0[1]*Wrist0[1]+Wrist0[0]*Wrist0[0]);
   if (Wrist0[1]==0&&Wrist0[0] - asin(l)==0) return false;
   qOut[0] = atan2(Wrist0[1],Wrist0[0]) - asin(l);
   if (elbow&0x01)    
	   qOut[0] = atan2(Wrist0[1],Wrist0[0]) - pi + asin(l);
   /////////////////////////////
   //Compute theta2 and theta3//
   /////////////////////////////
   double alpha;
   if (cos(qOut[0])==0) 
	   alpha = (Wrist0[1]-L1*cos(qOut[0]))/sin(qOut[0]);    
   else 
	   alpha = (Wrist0[0]+L1*sin(qOut[0]))/cos(qOut[0]);
   double beta = Wrist0[2];
   //We are seeking for a solution to the following equations:
   //   alpha = L2c2 + L3s(2+3) 
   //   beta  = -L2s2 + L3c(2+3)
   //   this gives --> alpha^2 + beta^2 = L2^2+L3^2 + 2L2L3(c2s(2+3)-s2c(2+3)) 
   //= L2^2+L3^2+2*L2L3s3 
   qOut[2] = asin ( (alpha*alpha+beta*beta-L2*L2-L3*L3)/(2*L2*L3) );
   if (elbow&0x02)
       qOut[2] = pi-qOut[2];
   //   The previous equations can be re-written as
   //   alpha*n = ct*c2 + st*s2 = c(t - 2)
   //   beta*n  = -ct*s2 + st*c2 = s(t - 2)
   double ct,st,n;
   ct = L2+L3*sin(qOut[2]); st = L3*cos(qOut[2]); n = sqrt(ct*ct+st*st); 
   if (n==0) return false;
   qOut[1] = atan2(st,ct) - atan2(beta,alpha);
   //////////////////////////////////
   //Compute theta4, theta5, theta6//
   //////////////////////////////////
   //The transformation from frame 3 to 7 T37 is easier to handle than T07 and we can obtain
   //the desired end-effector position and orientation in frame 3 and compare it to T37 to 
   //solve for theta5, theta6 and theta7
   //We start first by computing the desired position and orientation in frame3
   //To do this we need to compute T30 : we start computing T03
   PrMatrix T01(4,4), T12(4,4), T23(4,4), T03(4,4), T30(4,4);
   PrVector DH1 (4), DH2(4), DH3(4);
   DH1[0] = 0    ; DH1[1] = 0 ; DH1[2] = 0 ; DH1[3] = qOut[0];
   DH2[0] = -pi/2; DH2[1] = 0 ; DH2[2] = L1; DH2[3] = qOut[1];
   DH3[0] = 0    ; DH3[1] = L2; DH3[2] = 0 ; DH3[3] = qOut[2];
   DH_transform(DH1, T01);   DH_transform(DH2, T12);   DH_transform(DH3, T23);
   PrMatrix3 R03,R30; PrMatrix trans03(3,1),trans30(3,1);
   T03 = T01*T12*T23; T03.submatrix(0,0,R03); T03.submatrix(0,3,trans03);
   //Now let's invert T03
   R30 = R03.transpose(); trans30 = - R30*trans03;
   T30.setSubmatrix(0,0,R30); T30.setSubmatrix(0,3,trans30); T30[3][3] = 1;
   PrMatrix pos_rot_0 (4,4), pos_rot_3 (4,4);
   pos_rot_0.setSubmatrix(0,0,rot); pos_rot_0[3][3] = 1;
   pos_rot_0[0][3] = pos[0]; pos_rot_0[1][3] = pos[1]; pos_rot_0[2][3] = pos[2];
   //Now let's compute the translation-rotation matrix of the end-effector in frame 3
   pos_rot_3 = T30*pos_rot_0;
   //Compute theta4 from the position of the end-effector in frame 3
   qOut[3] = atan2(pos_rot_3[2][3],pos_rot_3[0][3]);
   if (elbow&0x04)
       qOut[3] = pi+qOut[3];
   //Compute theta5 from the position of the end-effector in frame 3
   if (pos_rot_3[2][3]==0&&pos_rot_3[0][3]==0) return false;
   if (pos_rot_3[0][3]!=0)
     qOut[4] = atan2(pos_rot_3[0][3]/L6/cos(qOut[3]),(pos_rot_3[1][3]+L3)/(-L6));
   if (pos_rot_3[1][0]==0&&pos_rot_3[1][1]==0) return false;
   //Compute theta4 from the orientation of the end-effector in frame 3 using direction cosines
   qOut[5] = atan2(-pos_rot_3[1][1],pos_rot_3[1][0]);
   if (elbow&0x04)
       qOut[5] = pi+qOut[5];
   return true;
}

void OpDynamics( const PrVector& fPrime, GlobalVariables& gv )
{
}

// *******************************************************************
// Helper Functions
// *******************************************************************

/*
 * Create a velocity saturated torque profile
 * returns torque vector
 */
PrVector6 velocitySaturate(GlobalVariables& gv) {
	PrVector6 tau_desired;
	tau_desired.zero();

	for(int i = 0; i < gv.kv.size(); i++) {
		if(gv.kv[i] == 0) {
			tau_desired[i] = -gv.kp[i] * (gv.q[i] - gv.qd[i]);
		} else {
			gv.dqd[i] = (gv.kp[i] / gv.kv[i]) * (gv.qd[i] - gv.q[i]);
			float nu = gv.dqmax[i] / abs(gv.dqd[i]);
			nu = abs(nu) > 1 ? (nu < 0 ? -1 : 1) : nu;
			tau_desired[i] = -gv.kv[i] * (gv.dq[i] - nu * gv.dqd[i]);
		}
	}
	return tau_desired;
}

/*
 * Computers the parameters for cubic spline trajectory
 * void
 */
void generateSplineTrajectory(GlobalVariables& gv) {
	spline.t0 = gv.curTime;
	if(InJointLimits(gv)) {
		spline.tf_max = 0;
		for(int i = 0; i < gv.qd.size(); i++) {
			float abs_diff = abs(gv.qd[i] - gv.q[i]);
			float tf = max((3.0 / (2.0 * gv.dqmax[i]) * abs_diff), (sqrt(6.0 / gv.ddqmax[i] * abs_diff)));
			if(tf > spline.tf_max)
				spline.tf_max = tf;
		}
		if(spline.tf_max > 0) { // prevent divide by zero
			spline.a0 = gv.q;
			spline.a2 = (gv.qd - gv.q) * (3.0 / pow(spline.tf_max, 2));
			spline.a3 = (gv.qd - gv.q) * (-2.0 / pow(spline.tf_max, 3));
			return;
		}
	}
	
  // no trajectory could be generated, ensure that the
  // robot does not move
	gv.qd = gv.q;
	spline.tf_max = -1;
}

/*
 * Tests if the given desired joint position is within joint limits
 */
bool InJointLimits(GlobalVariables& gv) {
	for(int i = 0; i < gv.qd.size(); i++) {
		if(gv.qd[i] > gv.qmax[i] || gv.qd[i] < gv.qmin[i]) {
			return false;
		}
	}
	return true;
}


#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32