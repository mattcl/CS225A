#ifndef CONTROL_DLL_H
#define CONTROL_DLL_H
  
#include "GlobalVariables.h"

#ifdef WIN32
# define _extern_marker_ 
#else  //#ifdef WIN32
# define _extern_marker_ extern
#endif //#ifdef WIN32

// Initialize your variables here.  InitControl() is called just once,
// before the first servo loop.  PreprocessControl() is called on
// every servo loop just before the control law function, and
// PostprocessControl() is called just after the control law function.

_extern_marker_ void InitControl(GlobalVariables& gv) ;
_extern_marker_ void PreprocessControl(GlobalVariables& gv) ;
_extern_marker_ void PostprocessControl(GlobalVariables& gv) ;

// Initialization functions for each control law.
// Called once when a new control mode is set.

_extern_marker_ void initFloatControl(GlobalVariables& gv) ;
_extern_marker_ void initNjholdControl(GlobalVariables& gv) ;
_extern_marker_ void initJholdControl(GlobalVariables& gv) ;
_extern_marker_ void initOpenControl(GlobalVariables& gv) ;
_extern_marker_ void initNjmoveControl(GlobalVariables& gv) ;
_extern_marker_ void initJmoveControl(GlobalVariables& gv) ;
_extern_marker_ void initNjgotoControl(GlobalVariables& gv) ;
_extern_marker_ void initJgotoControl(GlobalVariables& gv) ;
_extern_marker_ void initNjtrackControl(GlobalVariables& gv) ;
_extern_marker_ void initJtrackControl(GlobalVariables& gv) ;
_extern_marker_ void initNxtrackControl(GlobalVariables& gv) ;
_extern_marker_ void initXtrackControl(GlobalVariables& gv) ;
_extern_marker_ void initNholdControl(GlobalVariables& gv) ;
_extern_marker_ void initHoldControl(GlobalVariables& gv) ;
_extern_marker_ void initNgotoControl(GlobalVariables& gv) ;
_extern_marker_ void initGotoControl(GlobalVariables& gv) ;
_extern_marker_ void initNtrackControl(GlobalVariables& gv) ;
_extern_marker_ void initTrackControl(GlobalVariables& gv) ;
_extern_marker_ void initPfmoveControl(GlobalVariables& gv) ;
_extern_marker_ void initLineControl(GlobalVariables& gv) ;
_extern_marker_ void initProj1Control(GlobalVariables& gv) ;
_extern_marker_ void initProj2Control(GlobalVariables& gv) ;
_extern_marker_ void initProj3Control(GlobalVariables& gv) ;

// Functions containing the actual control laws.

_extern_marker_ void noControl(GlobalVariables& gv) ;
_extern_marker_ void floatControl(GlobalVariables& gv) ;
_extern_marker_ void njholdControl(GlobalVariables& gv) ;
_extern_marker_ void jholdControl(GlobalVariables& gv) ;
_extern_marker_ void openControl(GlobalVariables& gv) ;
_extern_marker_ void njmoveControl(GlobalVariables& gv) ;
_extern_marker_ void jmoveControl(GlobalVariables& gv) ;
_extern_marker_ void njgotoControl(GlobalVariables& gv) ;
_extern_marker_ void jgotoControl(GlobalVariables& gv) ;
_extern_marker_ void njtrackControl(GlobalVariables& gv) ;
_extern_marker_ void jtrackControl(GlobalVariables& gv) ;
_extern_marker_ void nxtrackControl(GlobalVariables& gv) ;
_extern_marker_ void xtrackControl(GlobalVariables& gv) ;
_extern_marker_ void nholdControl(GlobalVariables& gv) ;
_extern_marker_ void holdControl(GlobalVariables& gv) ;
_extern_marker_ void ngotoControl(GlobalVariables& gv) ;
_extern_marker_ void gotoControl(GlobalVariables& gv) ;
_extern_marker_ void ntrackControl(GlobalVariables& gv) ;
_extern_marker_ void trackControl(GlobalVariables& gv) ;
_extern_marker_ void pfmoveControl(GlobalVariables& gv) ;
_extern_marker_ void lineControl(GlobalVariables& gv) ;
_extern_marker_ void proj1Control(GlobalVariables& gv) ;
_extern_marker_ void proj2Control(GlobalVariables& gv) ;
_extern_marker_ void proj3Control(GlobalVariables& gv) ;

// PrintDebug is called when you type pdebug at the prompt
_extern_marker_ void PrintDebug(GlobalVariables& gv) ;



#endif