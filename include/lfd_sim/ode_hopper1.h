//-------------------------------------------------------------------------------------------
/*! \file    ode_hopper1.h
    \brief   Single-leg hopping robot simulator using ODE.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Dec.28, 2015
*/
//-------------------------------------------------------------------------------------------
#ifndef ode_hopper1_h
#define ode_hopper1_h
//-------------------------------------------------------------------------------------------
#include "lfd_sim/geom_util.h"
//-------------------------------------------------------------------------------------------
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawTriangle dsDrawTriangleD
#define dsDrawLine dsDrawLineD
#endif
//-------------------------------------------------------------------------------------------
#include <vector>
#include <valarray>
//-------------------------------------------------------------------------------------------
#ifndef override
#define override
#endif
//-------------------------------------------------------------------------------------------
namespace trick
{
//-------------------------------------------------------------------------------------------

namespace ode_x
{
//-------------------------------------------------------------------------------------------

struct TSensorsHp1;

extern double VizJointLen     ;
extern double VizJointRad     ;
extern double FSThick         ;
extern double FSSize          ;
extern double BaseLenX        ;
extern double BaseLenY        ;
extern double BaseLenZ        ;
extern double LegRad          ;
extern double LegLen1         ;
extern double LegLen2         ;
extern double EyeCylR         ;
extern double EyeCylL         ;
extern double FootRad         ;
extern double EyeRad          ;
extern double BaseDensity     ;
extern double OtherDensity    ;

extern int    TerrainMode     ;  // TerrainMode: 0: None, 1: Rough

extern int    MaxContacts     ;  // maximum number of contact points per body
extern double TimeStep        ;
extern double Gravity         ;
extern bool   EnableKeyEvent  ;  // If we use a default key events.

extern double HingeFMax       ;
extern int    ControlMode     ;  // 0: Position, 1: Velocity, 2: Torque/Force

namespace target
{
extern std::vector<double> Angles  ;
extern std::vector<double> Vel     ;
extern std::vector<double> Torque  ;
}

extern bool Running;
extern void (*SensingCallback)(const TSensorsHp1 &sensors);
extern void (*DrawCallback)(void);
extern void (*StepCallback)(const double &time, const double &time_step);
extern void (*KeyEventCallback)(int command);
//-------------------------------------------------------------------------------------------

// the following classes have a copy constructor and operator= that do nothing;
// these classes are defined in order to use std::vector of them
#define DEF_NC(x_class, x_col) \
  class TNC##x_class : public d##x_class  \
  {                                       \
  public:                                 \
    int ColorCode;                        \
    TNC##x_class() : d##x_class(), ColorCode(x_col) {} \
    TNC##x_class(const TNC##x_class&)     \
      : d##x_class(), ColorCode(x_col){}  \
    const TNC##x_class& operator=(const TNC##x_class&) {return *this;} \
  private:                                \
  };
DEF_NC(Body,0)
DEF_NC(BallJoint,0)
DEF_NC(HingeJoint,1)
DEF_NC(Hinge2Joint,2)
DEF_NC(SliderJoint,3)
DEF_NC(FixedJoint,4)
DEF_NC(Box,0)
DEF_NC(Capsule,1)
DEF_NC(Cylinder,2)
DEF_NC(Sphere,3)
#undef DEF_NC
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TTriMeshGeom : public dGeom
//===========================================================================================
{
public:
  int ColorCode;
  TTriMeshGeom() : dGeom(), ColorCode(4) {}

  TTriMeshGeom(const TTriMeshGeom&) : dGeom(), ColorCode(4) {}
  const TTriMeshGeom& operator=(const TTriMeshGeom&) {return *this;}

  dGeomID& create() {return _id;}

  void create(dSpaceID space)
    {
      dTriMeshDataID new_tmdata= dGeomTriMeshDataCreate();
      dGeomTriMeshDataBuildSingle(new_tmdata, &vertices_[0], 3*sizeof(float), vertices_.size()/3,
                  &indices_[0], indices_.size(), 3*sizeof(dTriIndex));
      _id= dCreateTriMesh(space, new_tmdata, NULL, NULL, NULL);
    }
  void setBody(dBodyID body)  {dGeomSetBody (_id,body);}

  const std::valarray<float>& getVertices() const {return vertices_;}
  const std::valarray<dTriIndex>& getIndices() const {return indices_;}

  std::valarray<float>& setVertices() {return vertices_;}
  std::valarray<dTriIndex>& setIndices() {return indices_;}

  void setVertices(float *array, int size)
    {
      vertices_.resize(size);
      for(float *itr(&vertices_[0]);size>0;--size,++itr,++array)
        *itr= *array;
    }
  void setIndices(dTriIndex *array, int size)
    {
      indices_.resize(size);
      for(dTriIndex *itr(&indices_[0]);size>0;--size,++itr,++array)
        *itr= *array;
    }

  void getMass(dMass &m, dReal density)
    {
      dMassSetTrimesh(&m, density, _id);
      printf("mass at %f %f %f\n", m.c[0], m.c[1], m.c[2]);
      dGeomSetPosition(_id, -m.c[0], -m.c[1], -m.c[2]);
      dMassTranslate(&m, -m.c[0], -m.c[1], -m.c[2]);
    }

protected:

  std::valarray<float> vertices_;
  std::valarray<dTriIndex> indices_;

};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TDynRobot
//===========================================================================================
{
public:

  std::vector<TNCBody>& Body() {return body_;}
  dBody& Body(int j) {return body_[j];}

  virtual void Create(dWorldID world, dSpaceID space) = 0;
  void Draw();

  const dReal GetAngleH(int j) const {return joint_h_[j].getAngle();}
  const dReal GetAngVelH(int j) const {return joint_h_[j].getAngleRate();}
  void SetVelH(int j, dReal vel)  {joint_h_[j].setParam(dParamVel,vel);}
  void AddTorqueH(int j, dReal tau)  {joint_h_[j].addTorque(tau);}

  const dReal GetPosS(int j) const {return joint_s_[j].getPosition();}
  const dReal GetVelS(int j) const {return joint_s_[j].getPositionRate();}
  void SetVelS(int j, dReal vel)  {joint_s_[j].setParam(dParamVel,vel);}
  void AddForceS(int j, dReal f)  {joint_s_[j].addForce(f);}

  const dJointFeedback& GetFeedback(int i) const {return feedback_[i];}

protected:
  std::vector<TNCBody> body_;
  std::vector<TNCBox> link_b_;
  std::vector<TNCCapsule> link_ca_;
  std::vector<TNCCylinder> link_cy_;
  std::vector<TNCSphere> link_sp_;
  std::vector<TTriMeshGeom> link_tm_;
  std::vector<TNCBallJoint> joint_b_;
  std::vector<TNCHingeJoint> joint_h_;
  std::vector<TNCHinge2Joint> joint_h2_;
  std::vector<TNCSliderJoint> joint_s_;
  std::vector<TNCFixedJoint> joint_f_;
  std::vector<dJointFeedback> feedback_;

  // Clear the elements.
  void clear(void)
    {
      body_.clear();
      link_b_.clear();
      link_ca_.clear();
      link_cy_.clear();
      link_sp_.clear();
      link_tm_.clear();
      joint_b_.clear();
      joint_h_.clear();
      joint_h2_.clear();
      joint_s_.clear();
      joint_f_.clear();
      feedback_.clear();
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class THopper1 : public TDynRobot
//===========================================================================================
{
public:
  override void Create(dWorldID world, dSpaceID space);
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TTerrain1 : public TDynRobot
//===========================================================================================
{
public:
  override void Create(dWorldID world, dSpaceID space);
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
struct TSensorsHp1
//===========================================================================================
{
  // Joint angles
  std::vector<double> JointAngles;
  // Poses (x,y,z,qx,qy,qz,qw) of links; [7]*(11)
  std::vector<double> LinkX;
  // Force/torque (fx,fy,fz,tx,ty,tz) of sensors and joints; [6]*(1+4)
  std::vector<double> Forces;
  // Masses of links; [11]
  std::vector<double> Masses;
  // Collision status (0: not colliding, 1: colliding)
  std::vector<int> Collisions;

  // Simulation time
  double Time;

  void Clear()
    {
      JointAngles.clear();
      LinkX.clear();
      Forces.clear();
      Masses.clear();
      Collisions.clear();
      SetZeros();
    }

  #define SETZERO(vec,type)       \
    for(std::vector<type>::iterator itr(vec.begin()),itr_end(vec.end());  \
        itr!=itr_end; ++itr)       \
      *itr= 0;
  #define SETZEROD(vec) SETZERO(vec,double)
  #define SETZEROI(vec) SETZERO(vec,int)
  void SetZeros()
    {
      JointAngles.resize(4);
      LinkX.resize(7*(11));
      Forces.resize(6*(1+4));
      Masses.resize(11);
      Collisions.resize(11);
      SETZEROD( JointAngles  )
      SETZEROD( LinkX        )
      SETZEROD( Forces       )
      SETZEROD( Masses       )
      SETZEROI( Collisions   )
      Time= 0.0;
    }

  // Reset values for each new physics computation step.
  // Only collision flags are reset. Other values are kept.
  void ResetForStep()
    {
      for(std::vector<int>::iterator itr(Collisions.begin()),itr_end(Collisions.end());
          itr!=itr_end; ++itr)
        *itr= 0;
    }
  #undef SETZERO
  #undef SETZEROD
  #undef SETZEROI
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
class TEnvironment
//===========================================================================================
{
public:
  TEnvironment()
    : space_(0) {}

  dWorldID WorldID() {return world_.id();}
  dSpaceID SpaceID() {return space_.id();}
  dJointGroupID ContactGroupID() {return contactgroup_.id();}

  // TDynRobot& Robot() {return robot_;}

  const dReal& Time() const {return time_;}

  void Create();
  void StepSim(const double &time_step);
  void Draw();

  void ControlCallback(const double &time_step);
  void EDrawCallback();
  /* Called when b1 and b2 are colliding.
      Return whether we ignore this collision (true: ignore collision). */
  bool CollisionCallback(dBodyID &b1, dBodyID &b2, dContact *contact);

private:
  dWorld world_;
  dSimpleSpace space_;
  dJointGroup contactgroup_;

  THopper1   hopper_;
  TTerrain1  terrain_;
  dPlane     plane_;

  dReal time_;

  TSensorsHp1  sensors_;
};
//-------------------------------------------------------------------------------------------


void Create();
void Reset();
void Run(int argc, char **argv, const char *texture_path="textures", int winx=500, int winy=400);
void Stop();
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
}  // end of ode_x
//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
#endif // ode_hopper1_h
//-------------------------------------------------------------------------------------------
