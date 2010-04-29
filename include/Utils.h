// *******************************************************************
// Utils.h
//
// This file declares miscellaneous functions used throughout the
// code.
// *******************************************************************

#ifndef CS225_UTILS_H
#define CS225_UTILS_H

#include <vector>
#include "PrVector.h"
#include "PrVector3.h"
#include "PrMatrix.h"

// angle conversion functions

Float toRad( Float val );
void  toRad( PrVector& vals );
Float toDeg( Float val );
void  toDeg( PrVector& vals );

// check joint limits

bool CheckJointLimits( const PrVector& pos,
                      const PrVector& qmin, const PrVector& qmax );

// shutdown

extern bool g_appQuit;  // Set by app_quit().  The various threads
// should quit when g_appQuit becomes true.
void app_quit();        // Quit function.  The simulator, rclient, and rt must
// all define their own app_quit()

#endif
