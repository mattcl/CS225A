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

struct cSplineParam {
	float tf_max;
	float t0;
	PrVector6 a0;
	PrVector6 a1;
	PrVector6 a2;
	PrVector6 a3;
};

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
	if(!inv_kin(gv.Td.translation(), gv.Td.rotation().matrix(), gv.elbow, gv.qd, gv)) {
		gv.qd = gv.q;
	}
}

void initXtrackControl(GlobalVariables& gv) 
{
	if(!inv_kin(gv.Td.translation(), gv.Td.rotation().matrix(), gv.elbow, gv.qd, gv)) {
		gv.qd = gv.q;
	}
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

bool inv_kin( const PrVector3& pos, const PrMatrix3& rot, int elbow,
              PrVector& qOut, GlobalVariables& gv )
{
	
	return true;
}

void OpDynamics( const PrVector& fPrime, GlobalVariables& gv )
{
}

// *******************************************************************
// Helper Functions
// *******************************************************************
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
		spline.a0 = gv.q;
		spline.a2 = (gv.qd - gv.q) * (3.0 / pow(spline.tf_max, 2));
		spline.a3 = (gv.qd - gv.q) * (-2.0 / pow(spline.tf_max, 3));
	} else {
		gv.qd = gv.q;
		spline.tf_max = -1;
	}
}

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