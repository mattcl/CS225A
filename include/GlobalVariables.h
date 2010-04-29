#ifndef DLL_SYNCH_DATA
#define DLL_SYNCH_DATA

#include "cs225.h"
#include "PrVector3.h"
#include "PrVector6.h"
#include "PrMatrix3.h"
#include "PrMatrix6.h"
#include "PrTransform.h"
#include "IRobotModel.h"

static const int MAX_OBSTACLES = 16;

struct GlobalVariables
{
  // main global vars
  // dof     : degrees of freedom
  // curTime : Our current simulator time
  // tau     : joint torques
  // q       : joint position
  // dq      : joint ang. velocity
  // kp      : position gains for the current control mode
  // kv      : velocity gains for the current control mode
  // x       : configuration parameters of end-effector in operational space
  // dx      : linear/angular velocity of end-effector in operational space
  // qd      : desired joint position
  // dqd     : desired joint ang. velocities
  // ddqd    : desired joint ang. acceleration
  // xd      : desired configuration parameters of end-effector (op.space)
  // dxd     : desired linear/angular velocity of end-effector (op.space)
  // ddxd    : desired linear/angular acceleration of end-effector (op.space)
  // elbow   : desired elbow (up/down, 0/1) for nxtrack, xtrack modes
  // T       : position/orientation of the end-effector (basically same as x)
  // Td      : desired position/orientation (basically same as xd)
  int         dof;
  Float       curTime;
  PrVector    tau;
  PrVector    q, dq, kp, kv, x, dx;
  PrVector    qd, dqd, ddqd, xd, dxd, ddxd;
  int         elbow;
  PrTransform T, Td;


  // Kinematics and Dynamics
  // J             : Current Jacobian
  // Jtranspose    : Jacobian transpose
  // A             : Mass matrix in joint space (sometimes called "M")
  // B             : Centrifugal/coriolis vector in joint space
  // G             : Gravity vector in joint space
  // Lambda        : Mass matrix in op. space
  // mu            : Centrifugal/coriolis vector in op. space
  // p             : Gravity vector in op. space
  // singularities : Indicates the current singularity(s)
  // E             : E*J gives rates in terms of configuration parameters
  // Einverse      : Inverse of the E matrix

  PrMatrix J, Jtranspose;
  PrMatrix A;
  PrVector B, mu;
  PrMatrix Lambda;
  PrVector G, p;
  int      singularities; // See HEAD_LOCK, ELBOW_LOCK, & WRIST_LOCK
  PrMatrix E, Einverse;


  // Limits
  // qmin, qmax    : min and max joint position values
  // dqmax, ddqmax : max joint angular velocities and accelerations
  // taumax          : max joint torques
  // xmin, xmax    : max positions in operational space
  // dxmax, ddxmax : max velocity & acceleration in operational space
  // wmax            : max angular velocity in operational space

  PrVector qmin, qmax, dqmax, ddqmax, taumax;
  PrVector xmin, xmax;
  Float    dxmax, ddxmax, wmax;


  // Potential fields
  // jlimit   : If true, activate soft joint limits
  // q0       : min. angular distance for joint-limit potential field
  // kj       : gain for joint-limit potential field
  // rho0     : min. distance for obstacle-avoidance potential field
  // eta      : gain for obstacle-avoidance potential field
  // sbound   : singularity bound
  // line     : line trajectory.  The "Line" type is declared in cs225.h.
  // numObstacles: Number of obstacles in the obstacles array.
  // obstacles: Array of obstacles.  The "Sphere" type is declared in cs225.h.

  bool     jlimit;
  PrVector q0, kj;
  Float    rho0, eta;
  Float    sbound;
  Line     line;
  int      numObstacles;
  Sphere   obstacles[MAX_OBSTACLES];


  // Selection matrices:
  // selectLinear:  Selects the linear components of dx
  // selectAngular: Selects the angular components of dx
  // selectLinearTranspose:  Transpose of selectLinear
  // selectAngularTranspose: Transpose of selectAngular

  PrMatrix selectLinear;
  PrMatrix selectAngular;
  PrMatrix selectLinearTranspose;
  PrMatrix selectAngularTranspose;

  // Miscellaneous
  // aKp     : position gains for all control modes
  // aKv     : velocity gains for all control modes
  // forces        : force-sensor measurement
  PrVector6 forces;
  PrVector  aKp[ NUM_CONTROL_MODES ];
  PrVector  aKv[ NUM_CONTROL_MODES ];
};

// Types of singularities.  singularities consists of these values OR-ed
// together.  singularites is 0 if the robot is not at a singularity.
//
// HEAD_LOCK : Singularity in which the wrist is over the base
// ELBOW_LOCK: Singularity in which the elbow (joint q3) is straight
// WRIST_LOCK: Singularity in which the finger (joint q5) is straight

static const int HEAD_LOCK  = IRobotModel::HEAD_LOCK;    // 0x01
static const int ELBOW_LOCK = IRobotModel::ELBOW_LOCK;   // 0x02
static const int WRIST_LOCK = IRobotModel::WRIST_LOCK;   // 0x04

#endif