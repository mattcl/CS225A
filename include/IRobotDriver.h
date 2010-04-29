// *******************************************************************
// IRobotDriver.h
//
// This class provides an abstract interface to a real or simulated
// robot.  All robots must have a simulation driver.  They only need a
// "real" driver if you plan to control a physical robot.
//
// The IRobotDriver interface takes care of the low-level robot
// control, such as reading velocities and setting torques.  See the
// IRobotModel interface for some highter-level details about how the
// robot is used.
//
// modification history
// --------------------
//
// 07/07/04: Dan Merget: created
// *******************************************************************
#ifndef I_ROBOT_DRIVER_H
#define I_ROBOT_DRIVER_H

#ifdef WIN32
#include <sys/types.h>
#include <time.h>
#else
#include <sys/times.h>
#endif
#include <vector>
#include "PrTransform.h"
class IRobotSimDriver;


// ===================================================================
// IRobotDriver: Abstract interface to a real or simulated robot.
//
// There are only a few methods that *have* to be implemented in a
// subclass, such as getPos().  Most of the methods are optional.
// (For example, readForceSensor() only applies to robots with a force
// sensor, and setFriction() only applies to simulated robots).  The
// optional methods have a NOP implementation in the base class.
// ===================================================================

class IRobotDriver
{
public:
  virtual ~IRobotDriver();

  // Get the current position, in joint space
  virtual void getPos( PrVector& dest ) = 0;

  // Get the current velocity, in joint space
  virtual void getVel( PrVector& dest ) = 0;

  // Set the torque
  virtual void setTorque( const PrVector& rhs ) = 0;

  // Wait until the next servo loop should begin.  simSpeed is used
  // in simulation mode (e.g. simSpeed = 0.5 runs at half speed).
  // The return value indicates the number of servo ticks that have
  // elapsed:
  //   2 or more = error; the servo loop is too slow
  //   1 = success
  //   0 = process a user command and then return to waitNextTick,
  //       without running a servo loop.  This return value only
  //       occurs in simulation, when simSpeed < 1.0.
  virtual int  waitNextTick( Float simSpeed ) = 0;

  // Read the force sensor
  virtual void readForceSensor( PrVector& dest );

  // Send a number to the gripper's DAC
  virtual void setGripperCommand( int cmd );

  // Reset the encoder
  virtual void resetEncoder();

  // Enable or disable friction (simulation only)
  virtual void setFriction( bool val );

  // Set the contact point used to apply contact force with the mouse
  // (simulation only).  linkNum is the link that was contacted (0 to
  // dof-1), contactPoint is the contact point in base coordinates,
  // and pos is the joint positions at the time contact was made.
  virtual void setContactPoint( int linkNum, const PrVector3& contactPoint,
    const PrVector& pos );

  // Apply a contact force to the contact point (simulation only).
  // The contact force equals springk * (destPoint - currentPoint).
  // Each time this function is called, it overrides the previous
  // call.  Must call setContactPoint first.
  virtual void applyContactForce( Float springk, const PrVector3& destPoint );

  // Remove the contact force applied by applyContactForce().
  virtual void removeContactForce();

  virtual void getJacobianAtContactPoint( PrMatrix& jacobian ){};

  virtual void getJacobianAt( int linkID, const PrVector3& localPos, PrMatrix& jacobian ){};

  virtual bool getGlobalTransformOf( int linkID, PrTransform& globalTransform ){ return false; };
};


// ===================================================================
// IRobotSimDriver: Abstract interface to a simulated robot.  This
// interface subclasses IRobotDriver.
// ===================================================================

class IRobotSimDriver : public IRobotDriver
{
public:
  IRobotSimDriver( int clockRate );

  // Default implementation of waitNextTick() that handles the timing
  // and the return value, but not the forward kinematics.
  virtual int  waitNextTick( Float simSpeed );

  // Set the position & velocity
  virtual void setState( const PrVector& pos, const PrVector& vel ) = 0;

  // Apply forward dynamics to convert a torque to an acceleration
  virtual void fwdDynamics( PrVector& acc, const PrVector& torque ) = 0;

  // Apply inverse dynamics to convert an acceleration to a torque
  virtual void invDynamics( PrVector& torque, const PrVector& acc ) = 0;

  // Get the position & orientation of the end-effector
  virtual void getEndEffectorFrame( PrTransform& dest ) = 0;

  // Extract the mass matrix from the simulated robot.  (This usually
  // takes too long to run every servo loop, but it's useful for
  // debugging.)
  virtual void getMassMatrix( PrMatrix& dest ) = 0;

  // Extract the gravity vector from the simulated robot.
  virtual void getGravityVector( PrVector& dest ) = 0;

protected:
  Float   m_secsPerTick;   // secs per servo loop
  Float   m_secsPerClock;  // secs per increment of clock()
  clock_t m_clockTime;     // most recent call of clock()
  int     m_holdCounter;   // used when we have to wait a long time
  Float   m_secsToWait;    // running total of how long to wait
};

#endif // I_ROBOT_DRIVER_H
