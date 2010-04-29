// *******************************************************************
// IRobotModel.h
//
// This class provides an abstract interface to a "robot model": a
// collection of parameters that tells how a robot will be used.  A
// robot can have many models associated with it.  The model contains
// information such as:
//
//  - Which joints will be used, and should be "frozen" in order
//    to simplify the interface.
//  - Calculating the operational-space coordinates.  This includes
//    answering questions such as, "Will I represent the orientation
//    as euler angles, quaternions, or direction cosines?"  Different
//    models may have different answers.
//  - Joint limits, maximum velocity, suggested control gains, etc.
//
// An IRobotModel object does not do any robot control or simulation;
// see the IRobotDriver class for that.
//
// modification history
// --------------------
//
// 07/07/04: Dan Merget: created
// *******************************************************************
#ifndef I_ROBOT_MODEL_H
#define I_ROBOT_MODEL_H

#include <vector>
#include "PrVector.h"
#include "PrMatrix.h"
#include "PrTransform.h"
#include "IRobotDriver.h"

class IRobotModel
{
public:
  virtual ~IRobotModel() {}

  // Degrees of freedom of the robot, not including the frozen
  // joints.  (For example, if a puma has 6 joints, but 5 are frozen
  // in this model, then dof() returns 1).
  int dof() const { return m_dof; }

  // Degrees of freedom in this robot, including the frozen joints.
  int fullDof() const { return m_fullDof; }

  // Degrees of freedom of the end-effector
  int eeDof() const { return m_eeDof; }

  // Joint limits.
  const PrVector& qmin()   const { return m_qmin; }
  const PrVector& qmax()   const { return m_qmax; }
  const PrVector& dqmax()  const { return m_dqmax; }
  const PrVector& ddqmax() const { return m_ddqmax; }
  const PrVector& taumax() const { return m_taumax; }

  // Maximum linear velocity, linear accelelation, and angular
  // velocity in operational space
  Float dxmax()  const { return m_dxmax; }
  Float ddxmax() const { return m_ddxmax; }
  Float wmax()   const { return m_wmax; }

  // Suggested control gains
  const PrVector& suggested_kp()  const { return m_kp; }
  const PrVector& suggested_kv()  const { return m_kv; }
  const PrVector& suggested_kpx() const { return m_kpx; }
  const PrVector& suggested_kvx() const { return m_kvx; }

  // jointSelectionMatrix() is a dof x fullDof selection matrix that
  // selects the "unfrozen" joints in the robot.
  const PrMatrix& jointSelectionMatrix() const { return m_jointSelectionMatrix; }

  // linearSelectionMatrix and angularSelectionMatrix are designed to
  // separate the operational-space force or velocity into linear
  // components and angular components.
  //
  // For example, in a planar robot with coordinates (x, y, alpha),
  // linearSelectionMatrix() would be
  //   [ 1 0 0 ]
  //   [ 0 1 0 ]
  // and angularSelectionMatrix() would be
  //   [ 0 0 1 ]
  const PrMatrix& linearSelectionMatrix()  const { return m_linearSelectionMatrix; }
  const PrMatrix& angularSelectionMatrix() const { return m_angularSelectionMatrix; }

  // homePosition() is a vector with fullDof elements, which
  // specifies the "home" position of all joints.  This should only
  // affect the "frozen" joints; the home positions of the "unfrozen"
  // joints should be zero.
  const PrVector& homePosition()    const { return m_homePosition; }
  const PrVector& homePositionDeg() const { return m_homePositionDeg; }

  // Given the full robot state (with fullDof joints), calculate the
  // various vectors & matrices needed to implement joint-space
  // control or to calculate operational-space dynamics.  Unless
  // otherwise noted, the outputs are dof x 1 vectors or dof x dof
  // matrices.
  virtual void analyzeState(
    IRobotSimDriver* evalRobot,   // Simulation robot, with state set (input)
    Float sbound,             // Singularity bound (input)
    PrVector& my_q,           // Joint position
    PrVector& my_dq,          // Joint velocity
    PrMatrix& my_J,           // Jacobian
    PrMatrix& my_dJ,          // Derivative of Jacobian vs. time
    PrMatrix& my_A,           // Mass matrix
    PrVector& my_B,           // Centrifugal/coriolis vector
    PrVector& my_G,           // Gravity vector
    int& my_singularities,    // OR of bits in Singularities
    PrVector& my_nullSpaceTau // fullDof x 1 torque to "freeze" unused joints
    ) const = 0;

  // This section relates to the configuration parameters, used to
  // represent the configuration in operational space.

  int numConfigParms() const { return m_numConfigParms; }
  const char* configParmName( int index ) const { return m_configParmNames[index]; }
  const PrVector& xmin() const { return m_xmin; }
  const PrVector& xmax() const { return m_xmax; }
  virtual void getOperationalSpace( IRobotSimDriver* evalRobot,
    const PrVector& my_q,
    PrVector& my_x,
    PrMatrix& my_E,
    PrMatrix& my_Einverse ) const = 0;
  void xToDeg( const PrVector& src, PrVector& dest ) const;
  void xToRad( const PrVector& src, PrVector& dest ) const;
  virtual void getFrame( const PrVector& x, PrTransform& frame ) const = 0;
  virtual void getConfigParms( const PrTransform& frame, PrVector& x ) const = 0;
  virtual void normalizeConfigParms( PrVector& x ) const;

  // Singularity bits set in analyzeState()
  enum Singularities {
    HEAD_LOCK = 0x01,
    ELBOW_LOCK = 0x02,
    WRIST_LOCK = 0x04
  };

public:
  // Used to implement constructor, where fullModel represents the
  // full model with m_fullDof joints, and the bulk of this model can
  // be constructed by paring down the full model.
  void constructSubmodel( const IRobotModel& fullModel,
    int dof, int eeDof, int numConfigParms,
    int numLinearParms,
    const Float* jointSelectionMatrix,
    const Float* linearSelectionMatrix,
    const Float* angularSelectionMatrix,
    const Float* homePosition,
    const PrMatrix jacobianSelectionMatrix,
    const int* correspondingConfigParms );

  // Used to implement analyzeState, where fullModel represents the
  // full model with m_fullDof joints.  Most of the outputs are created
  // by multiplying the full results by m_jointSelectionMatrix.
  void analyzeSubstate( const IRobotModel& fullModel,
    IRobotSimDriver* evalRobot, Float sbound,
    PrVector& my_q, PrVector& my_dq,
    PrMatrix& my_J, PrMatrix& my_dJ,
    PrMatrix& my_A, PrVector& my_B, PrVector& my_G,
    int& my_singularities,
    PrVector& my_nullSpaceTau ) const;

  // These should be set in the constructor, and left alone
  // thereafter
  int m_dof;
  int m_fullDof;
  int m_eeDof;
  PrVector m_qmin;
  PrVector m_qmax;
  PrVector m_dqmax;
  PrVector m_ddqmax;
  PrVector m_taumax;
  Float m_dxmax;
  Float m_ddxmax;
  Float m_wmax;
  PrVector m_kp;
  PrVector m_kv;
  PrVector m_kpx;
  PrVector m_kvx;
  PrMatrix m_jointSelectionMatrix;
  PrMatrix m_linearSelectionMatrix;
  PrMatrix m_angularSelectionMatrix;
  PrVector m_homePosition;
  PrVector m_homePositionDeg;

  int m_numConfigParms;
  std::vector<const char*> m_configParmNames;
  PrVector m_xmin;
  PrVector m_xmax;
  std::vector<bool> m_angularConfigParms; // Array of size m_numConfigParms,
  // indicating which config parameters
  // should be converted by xToDeg & xToRad
};

#endif // I_ROBOT_MODEL_H
