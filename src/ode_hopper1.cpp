//-------------------------------------------------------------------------------------------
/*! \file    ode_hopper1.cpp
    \brief   Single-leg hopping robot simulator using ODE.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Dec.28, 2015
*/
//-------------------------------------------------------------------------------------------
#include "lfd_sim/ode_hopper1.h"
#include <iostream>
#include <cassert>
//-------------------------------------------------------------------------------------------
namespace trick
{
using namespace std;
// using namespace boost;


//-------------------------------------------------------------------------------------------
namespace ode_x
{

static const int MAX_CONTACTS_CAPACITY(10);  // MaxContacts can not be greater than this.

//-------------------------------------------------------------------------------------------
double VizJointLen     (0.1)  ;
double VizJointRad     (0.01) ;
double FSThick         (0.001);
double FSSize          (0.004);
double BaseLenX        (0.20) ;
double BaseLenY        (0.30) ;
double BaseLenZ        (0.15) ;
double LegRad          (0.02) ;
double LegLen1         (0.04) ;
double LegLen2         (0.20) ;
double EyeCylR         (0.01) ;
double EyeCylL         (0.08) ;
double FootRad         (0.08) ;
double EyeRad          (0.04) ;
double BaseDensity     (500.0)  ;
double OtherDensity    (100.0)  ;

int    TerrainMode     (0)    ;  // TerrainMode: 0: None, 1: Rough

int    MaxContacts     (4)    ;  // maximum number of contact points per body
double TimeStep        (0.005);
double Gravity         (-9.8) ;
bool   EnableKeyEvent  (true) ;  // If we use a default key events.

double HingeFMax       (100.0);
int    ControlMode     (0)    ;  // 0: Position, 1: Velocity, 2: Torque/Force
namespace target
{
std::vector<double> Angles  (4,0.0);
std::vector<double> Vel     (4,0.0);
std::vector<double> Torque  (4,0.0);
}

bool Running(true);
void (*SensingCallback)(const TSensorsHp1 &sensors)= NULL;
void (*DrawCallback)(void)= NULL;
void (*StepCallback)(const double &time, const double &time_step)= NULL;
void (*KeyEventCallback)(int command)= NULL;
//-------------------------------------------------------------------------------------------

static TEnvironment *Env(NULL);
static void NearCallback(void*,dGeomID,dGeomID);
//-------------------------------------------------------------------------------------------


dReal IndexedColors[][4]= {
    /*0*/ {1.0, 0.0, 0.0, 0.8},
          {0.0, 1.0, 0.0, 0.8},
          {0.0, 0.0, 1.0, 0.8},
    /*3*/ {1.0, 1.0, 0.0, 0.8},
          {1.0, 0.0, 1.0, 0.8},
          {0.0, 1.0, 1.0, 0.8},
    /*6*/ {1.0, 0.0, 0.0, 0.3},
          {0.0, 1.0, 0.0, 0.3},
          {0.0, 0.0, 1.0, 0.3},
    /*9*/ {1.0, 1.0, 0.0, 0.3},
          {1.0, 0.0, 1.0, 0.3},
          {0.0, 1.0, 1.0, 0.3},
    /*12*/{1.0, 1.0, 1.0, 0.8}};
inline void SetColor(int i)
{
  dReal *col= IndexedColors[i];
  dsSetColorAlpha(col[0],col[1],col[2],col[3]);
}
//-------------------------------------------------------------------------------------------

/* ODE::dBody pose to pose.
    x: [0-2]: position x,y,z, [3-6]: orientation x,y,z,w. */
template <typename t_array>
inline void ODEBodyToX(const dBody &body, t_array x)
{
  const dReal *p= body.getPosition();  // x,y,z
  const dReal *q= body.getQuaternion();  // qw,qx,qy,qz
  x[0]= p[0]; x[1]= p[1]; x[2]= p[2];
  x[3]= q[1]; x[4]= q[2]; x[5]= q[3]; x[6]= q[0];
}
//-------------------------------------------------------------------------------------------

/* ODE::dJointFeedback to force/torque.
    f: [0-2]: force, [3-5]: torque. */
template <typename t_array>
inline void ODEFeedbackToF(const dJointFeedback &feedback, t_array f)
{
  f[0]= feedback.f1[0]; f[1]= feedback.f1[1]; f[2]= feedback.f1[2];
  f[3]= feedback.t1[0]; f[4]= feedback.t1[1]; f[5]= feedback.t1[2];
}
//-------------------------------------------------------------------------------------------



//===========================================================================================
// class TDynRobot
//===========================================================================================

void TDynRobot::Draw()
{
  dsSetTexture (DS_WOOD);

  dReal rad, len;
  dReal sides[4];
  for (std::vector<TNCBox>::const_iterator itr(link_b_.begin()),last(link_b_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getLengths(sides);
    dsDrawBox (itr->getPosition(), itr->getRotation(), sides);
  }
  for (std::vector<TNCSphere>::const_iterator itr(link_sp_.begin()),last(link_sp_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    dsDrawSphere (itr->getPosition(), itr->getRotation(), itr->getRadius());
  }
  for (std::vector<TNCCapsule>::const_iterator itr(link_ca_.begin()),last(link_ca_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getParams(&rad, &len);
    dsDrawCapsule (itr->getPosition(), itr->getRotation(), len,rad);
  }
  for (std::vector<TNCCylinder>::const_iterator itr(link_cy_.begin()),last(link_cy_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getParams(&rad, &len);
    dsDrawCylinder (itr->getPosition(), itr->getRotation(), len,rad);
  }
  for (std::vector<TTriMeshGeom>::const_iterator itr(link_tm_.begin()),last(link_tm_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    // const dReal *pos = itr->getPosition();
    // const dReal *rot = itr->getRotation();
    const dVector3 pos={0,0,0,0};
    const dMatrix3 rot={1,0,0,0 ,0,1,0,0, 0,0,1,0};

    for (int i(dGeomTriMeshGetTriangleCount(*itr)); i>0; --i)
    {
      std::cerr<<"i:"<<i<<std::endl;
      dVector3 v[3];
      dGeomTriMeshGetTriangle(*itr, i-1, &v[0], &v[1], &v[2]);
      std::cerr<<v[0][0]<<" "<<v[0][1]<<" "<<v[0][2]<<std::endl;
      std::cerr<<v[1][0]<<" "<<v[1][1]<<" "<<v[1][2]<<std::endl;
      std::cerr<<v[2][0]<<" "<<v[2][1]<<" "<<v[2][2]<<std::endl;
      dsDrawTriangle(pos, rot, v[0], v[1], v[2], 1);
    }
  }

  dVector3 pos,axis;
  dMatrix3 rot;
  for (std::vector<TNCHingeJoint>::const_iterator itr(joint_h_.begin()),last(joint_h_.end()); itr!=last; ++itr)
  {
    SetColor(itr->ColorCode);
    itr->getAnchor(pos);
    itr->getAxis(axis);
    dRFromAxisAndAngle (rot,-axis[1],axis[0],0.0, 0.5*M_PI-std::asin(axis[2]));
    dsDrawCylinder (pos, rot, VizJointLen,VizJointRad);
  }

  // for (std::vector<TNCSliderJoint>::const_iterator itr(joint_s_.begin()),last(joint_s_.end()); itr!=last; ++itr)
  // {
    // SetColor(itr->ColorCode);
    // // itr->getAnchor(pos);
    // const dReal *p1(dBodyGetPosition(itr->getBody(0))), *p2(dBodyGetPosition(itr->getBody(1)));
    // for(int d(0); d<3; ++d)  pos[d]= 0.5*(p1[d]+p2[d]);
    // itr->getAxis(axis);
    // dRFromAxisAndAngle (rot,-axis[1],axis[0],0.0, 0.5*M_PI-std::asin(axis[2]));
    // dsDrawCylinder (pos, rot, VizJointLen, VizJointRad);
  // }
}
//-------------------------------------------------------------------------------------------

//===========================================================================================
// class THopper1 : public TDynRobot
//===========================================================================================

/*override*/void THopper1::Create(dWorldID world, dSpaceID space)
{
  clear();

  body_.resize(11);
  link_b_.resize(/*base:*/1 + /*force sensors:*/1);
  link_cy_.resize(/*leg:*/4+/*eye:*/2);
  link_sp_.resize(/*foot:*/1+/*eye:*/2);
  int ib(0);  // body counter

  dReal f_footsp(0.2);
  dReal f_eyesp(0.2);

  // create boxes
  dReal base_h= 0.5*BaseLenZ+FSThick+2.0*LegLen1+2.0*LegLen2+(1.0-f_footsp)*2.0*FootRad;
  dReal box_dns_size_pos[2][7]={
      // {Density, Lx,Ly,Lz, X,Y,Z}
      // base:
      /* 0*/{BaseDensity, BaseLenX,BaseLenY,BaseLenZ,  0.0,0.0,base_h},
      // force sensor:
      /* 1*/{OtherDensity, FSSize,FSSize,FSThick,  0.0,0.0,base_h-0.5*BaseLenZ-0.5*FSThick}
    };
  for(int i(0); i<2; ++i,++ib)
  {
    dReal density(box_dns_size_pos[i][0]);
    dReal *size(box_dns_size_pos[i]+1), *pos(box_dns_size_pos[i]+4);
    body_[ib].create(world);
    body_[ib].setPosition(pos[0],pos[1],pos[2]);
    dMass m;
    m.setBox(density,size[0],size[1],size[2]);
    // m.adjust(mass);
    body_[ib].setMass(&m);
    link_b_[i].create(space,size[0],size[1],size[2]);
    link_b_[i].setBody(body_[ib]);
  }

  // create cylinders
  dReal leg_h0= base_h-0.5*BaseLenZ-FSThick;
  dReal cyl_size_pos[6][5]={
      // {Rad,Len, X,Y,Z}
      // leg:
      /* 2*/{LegRad,LegLen1,  0.0,0.0,leg_h0-0.5*LegLen1},
      /* 3*/{LegRad,LegLen1,  0.0,0.0,leg_h0-1.5*LegLen1},
      /* 4*/{LegRad,LegLen2,  0.0,0.0,leg_h0-2.0*LegLen1-0.5*LegLen2},
      /* 5*/{LegRad,LegLen2,  0.0,0.0,leg_h0-2.0*LegLen1-1.5*LegLen2},
      // eye:
      /* 6*/{EyeCylR,EyeCylL,  0.4*BaseLenX,+0.3*BaseLenY,base_h+0.5*BaseLenZ+0.5*EyeCylL},
      /* 7*/{EyeCylR,EyeCylL,  0.4*BaseLenX,-0.3*BaseLenY,base_h+0.5*BaseLenZ+0.5*EyeCylL}
    };
  for(int i(0); i<6; ++i,++ib)
  {
    dReal *rl(cyl_size_pos[i]), *pos(cyl_size_pos[i]+2);
    link_cy_[i].create(space,/*radius*/rl[0],/*length*/rl[1]);
    body_[ib].create(world);
    body_[ib].setPosition(pos[0],pos[1],pos[2]);
    dMass m;
    // m.setCylinder(OtherDensity,/*z*/3,/*radius*/rl[0],/*length*/rl[1]);
    dMassSetCylinder(&m,OtherDensity,/*z*/3,/*radius*/rl[0],/*length*/rl[1]);
    body_[ib].setMass(&m);
    link_cy_[i].setBody(body_[ib]);
    link_cy_[i].ColorCode= i%6;
  }

  // create spheres
  dReal sp_size_pos[6][4]={
      // {Rad, X,Y,Z}
      // foot:
      /* 8*/{FootRad,  0.0,0.0,leg_h0-2.0*LegLen1-2.0*LegLen2-(1.0-f_footsp*2.0)*FootRad},
      // eye:
      /* 9*/{EyeRad,  0.45*BaseLenX,+0.3*BaseLenY,base_h+0.5*BaseLenZ+EyeCylL+f_eyesp*EyeRad},
      /*10*/{EyeRad,  0.45*BaseLenX,-0.3*BaseLenY,base_h+0.5*BaseLenZ+EyeCylL+f_eyesp*EyeRad}
    };
  for(int i(0); i<3; ++i,++ib)
  {
    dReal *rad(sp_size_pos[i]), *pos(sp_size_pos[i]+1);
    link_sp_[i].create(space, rad[0]);
    body_[ib].create(world);
    body_[ib].setPosition(pos[0],pos[1],pos[2]);
    dMass m;
    m.setSphere(OtherDensity, rad[0]);
    body_[ib].setMass(&m);
    link_sp_[i].setBody(body_[ib]);
    link_sp_[i].ColorCode= i;
  }


  joint_h_.resize(/*leg:*/4);
  joint_f_.resize(6);
  feedback_.resize(/*force sensors:*/1 + /*joint force sensors:*/4);
  int ifb(0);  // feedback counter

  dBodyID fixed_idxs[6][2]={
      // force sensor:
      /* 0*/{body_[0],body_[1]},
      // fixing foot, eyes:
      /* 1*/{body_[5],body_[8]},
      /* 2*/{body_[0],body_[6]},
      /* 3*/{body_[0],body_[7]},
      /* 4*/{body_[6],body_[9]},
      /* 5*/{body_[7],body_[10]}
    };
  for(int j(0),j_end(joint_f_.size()); j<j_end; ++j)
  {
    joint_f_[j].create(world);
    joint_f_[j].attach(fixed_idxs[j][0],fixed_idxs[j][1]);
    joint_f_[j].set();
  }
  int fs_idxs[1]= {0};
  for(int i(0); i<1; ++i,++ifb)
  {
    dJointSetFeedback(joint_f_[fs_idxs[i]], &feedback_[ifb]);
  }

  dReal fmax(HingeFMax);  // NOTE: set zero to control by joint torque
  if(ControlMode==2)  fmax= 0.0;  // Torque/Force control
  int jh_idx_ax[4][3]={
      // body index-1,-2, axis (0:x,1:y,2:z)
      // leg:
      /* 0*/{1,2, 2},
      /* 1*/{2,3, 0},
      /* 2*/{3,4, 1},
      /* 3*/{4,5, 1}
    };
  dReal jh_anch[4][3]={
      /* 0*/{0.0,0.0,leg_h0},
      /* 1*/{0.0,0.0,leg_h0-LegLen1},
      /* 2*/{0.0,0.0,leg_h0-2.0*LegLen1},
      /* 3*/{0.0,0.0,leg_h0-2.0*LegLen1-LegLen2}
    };
  for(int j(0); j<4; ++j,++ifb)
  {
    joint_h_[j].create(world);
    joint_h_[j].attach(body_[jh_idx_ax[j][0]],body_[jh_idx_ax[j][1]]);
    joint_h_[j].setAnchor(jh_anch[j][0], jh_anch[j][1], jh_anch[j][2]);
    switch(jh_idx_ax[j][2])
    {
    case 0: joint_h_[j].setAxis(1.0,0.0,0.0); break;
    case 1: joint_h_[j].setAxis(0.0,1.0,0.0); break;
    case 2: joint_h_[j].setAxis(0.0,0.0,1.0); break;
    }
    joint_h_[j].setParam(dParamFMax,fmax);
    dJointSetFeedback(joint_h_[j], &feedback_[ifb]);
  }
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TTerrain1 : public TDynRobot
//===========================================================================================

/*override*/void TTerrain1::Create(dWorldID world, dSpaceID space)
{
  clear();

  // body_.resize(2);
  // link_b_.resize(2);
  // int ib(0);  // body counter
}
//-------------------------------------------------------------------------------------------


//===========================================================================================
// class TEnvironment
//===========================================================================================

void TEnvironment::Create()
{
  contactgroup_.create();
  world_.setGravity(0,0,Gravity);
  dWorldSetCFM(world_.id(),1e-5);
  plane_.create(space_,0,0,1,0);

  hopper_.Create(world_,space_);
  // TerrainMode: 0: None, 1: Rough
  if(TerrainMode==1)  terrain_.Create(world_,space_);

  time_= 0.0;
  sensors_.Clear();
  sensors_.SetZeros();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::StepSim(const double &time_step)
{
  ControlCallback(time_step);

  sensors_.ResetForStep();

  space_.collide(0,&NearCallback);
  world_.step(time_step);
  time_+= time_step;

  contactgroup_.empty();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::Draw()
{
  if(Running)  EDrawCallback();

  hopper_.Draw();
  // TerrainMode: 0: None, 1: Rough
  if(TerrainMode==1)  terrain_.Draw();
}
//-------------------------------------------------------------------------------------------

void TEnvironment::ControlCallback(const double &time_step)
{
  // 0: Position, 1: Velocity, 2: Torque/Force
  if(ControlMode==0)  // Position control
  {
    dReal Kp(10.0);
    for(int j(0); j<4; ++j)
      hopper_.SetVelH(j, Kp*(target::Angles[j]-hopper_.GetAngleH(j)));
  }
  else if(ControlMode==1)  // Velocity control
  {
    for(int j(0); j<4; ++j)
      hopper_.SetVelH(j, target::Vel[j]);
  }
  else if(ControlMode==2)  // Torque/Force control
  {
    for(int j(0); j<4; ++j)
      hopper_.AddTorqueH(j, target::Torque[j]);
  }
  /*TEST of getForce: it gives the force added by user (not actually simulated force); i.e. useless in general.
  dVector3 f;
  int i=1;
  std::cerr<<"F= "<<hopper_.Body(i).getForce()[0]<<", "<<hopper_.Body(i).getForce()[1]<<", "<<hopper_.Body(i).getForce()[2]<<std::endl;
  //*/
  /*TEST of dJointSetFeedback
  std::cerr<<"F= "<<joint_feedback1.f1[0]<<", "<<joint_feedback1.f1[1]<<", "<<joint_feedback1.f1[2]<<std::endl;
  std::cerr<<"T= "<<joint_feedback1.t1[0]<<", "<<joint_feedback1.t1[1]<<", "<<joint_feedback1.t1[2]<<std::endl;
  //*/
}
//-------------------------------------------------------------------------------------------

void TEnvironment::EDrawCallback()
{
  for(int j(0); j<4; ++j)
    sensors_.JointAngles[j]= hopper_.GetAngleH(j);

  for(int i(0); i<11; ++i)
    ODEBodyToX(hopper_.Body(i), sensors_.LinkX.begin()+7*i);

  for(int i(0); i<1+4; ++i)
    ODEFeedbackToF(hopper_.GetFeedback(i), sensors_.Forces.begin()+6*i);

  for(int i(0); i<11; ++i)
    sensors_.Masses[i]= hopper_.Body(i).getMass().mass;

  sensors_.Time= time_;

  if(SensingCallback!=NULL)  SensingCallback(sensors_);
  if(DrawCallback!=NULL)  DrawCallback();
}
//-------------------------------------------------------------------------------------------

/* Called when b1 and b2 are colliding.
    Return whether we ignore this collision (true: ignore collision). */
bool TEnvironment::CollisionCallback(dBodyID &b1, dBodyID &b2, dContact *contact)
{
  // Check body collision
  // TODO: Use a map [id]-->index to speed up this search
  int idx(0);
  for(std::vector<TNCBody>::const_iterator itr(hopper_.Body().begin()),itrend(hopper_.Body().end());
      itr!=itrend; ++itr,++idx)
  {
    if(b1==itr->id() || b2==itr->id())
    {
      if(b1==0 || b2==0)  sensors_.Collisions[idx]= 1;  // Colliding with ground
      else  sensors_.Collisions[idx]= 1;
    }
  }
  return false;
}
//-------------------------------------------------------------------------------------------


//===========================================================================================


static void NearCallback(void *data, dGeomID o1, dGeomID o2)
{
  assert(Env!=NULL);

  // do nothing if the two bodies are connected by a joint
  dBodyID b1= dGeomGetBody(o1);
  dBodyID b2= dGeomGetBody(o2);
  if(b1 && b2 && dAreConnected(b1,b2)) return;

  if(MaxContacts>MAX_CONTACTS_CAPACITY)  MaxContacts= MAX_CONTACTS_CAPACITY;
  // std::valarray<dContact> contact(MaxContacts);   // up to MaxContacts contacts per link
  dContact contact[MAX_CONTACTS_CAPACITY+2];  //+2 is to avoid an ODE bug
  for(int i(0); i<MaxContacts; ++i)
  {
    contact[i].surface.mode= dContactBounce | dContactSoftCFM;
    contact[i].surface.mu= 100.0;  // dInfinity;
    contact[i].surface.mu2= 0.1;
    contact[i].surface.bounce= 0.1;
    contact[i].surface.bounce_vel= 0.01;
    contact[i].surface.soft_cfm= 0.005;
  }
  if(int numc=dCollide(o1,o2,MaxContacts,&contact[0].geom,sizeof(dContact)))
  {
    if(Env->CollisionCallback(b1,b2,contact))  return;  // ignore if the callback returns true
    for(int i(0); i<numc; ++i)
    {
      dJointID c= dJointCreateContact(Env->WorldID(),Env->ContactGroupID(),&contact[i]);
      dJointAttach (c,b1,b2);
    }
  }
}
//-------------------------------------------------------------------------------------------

static void SimStart()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {0.0,1.2447,0.7};
  static float hpr[3] = {-90.0000,-7.0000,0.0000};
  dsSetViewpoint (xyz,hpr);
}
//-------------------------------------------------------------------------------------------

static void SimLoop(int pause)
{
  assert(Env!=NULL);

  if (!pause && Running)
  {
    Env->StepSim(TimeStep);
  }

  Env->Draw();
  if(StepCallback!=NULL)  StepCallback(Env->Time(), TimeStep);
}
//-------------------------------------------------------------------------------------------

static void SimKeyevent(int command)
{
  assert(Env!=NULL);

  if(KeyEventCallback!=NULL)  KeyEventCallback(command);
  if(EnableKeyEvent)
  {
    dReal diff(0.2);
    switch(command)
    {
    case 'r':
    case 'R': Create(); break;
    case ' ': Running= !Running; break;
    case 'z':
      for(int j(0); j<4; ++j)  target::Angles[j]+= diff;
      break;
    case 'x':
      for(int j(0); j<4; ++j)  target::Angles[j]-= diff;
      break;
    case 'a':  target::Angles[0]+= diff;  break;
    case 'A':  target::Angles[0]-= diff;  break;
    case 's':  target::Angles[1]+= diff;  break;
    case 'S':  target::Angles[1]-= diff;  break;
    case 'd':  target::Angles[2]+= diff;  break;
    case 'D':  target::Angles[2]-= diff;  break;
    case 'f':  target::Angles[3]+= diff;  break;
    case 'F':  target::Angles[3]-= diff;  break;
    // case 'g':  target::Angles[4]+= diff;  break;
    // case 'h':  target::Angles[4]-= diff;  break;
    // case 'G':  target::Angles[5]+= diff;  break;
    // case 'H':  target::Angles[5]-= diff;  break;
    // case ']':  target::GPos[0]+= 0.002; target::GPos[1]+= 0.002;  break;
    // case '[':  target::GPos[0]-= 0.002; target::GPos[1]-= 0.002;  break;
    }
  }
}
//-------------------------------------------------------------------------------------------

void Create()
{
  assert(Env!=NULL);
  target::Angles.resize(4);
  target::Vel.resize(4);
  target::Torque.resize(4);
  for(int j(0); j<4; ++j)
  {
    target::Angles[j]= 0.0;
    target::Vel[j]= 0.0;
    target::Torque[j]= 0.0;
  }
  Env->Create();
}
//-------------------------------------------------------------------------------------------

void Reset()
{
  Create();
}
//-------------------------------------------------------------------------------------------

void Run(int argc, char **argv, const char *texture_path, int winx, int winy)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &SimStart;
  fn.step = &SimLoop;
  fn.command = &SimKeyevent;
  fn.stop = 0;
  fn.path_to_textures = texture_path;

  dInitODE2(0);

  TEnvironment env;
  Env= &env;
  // env.Create();
  Create();

  dsSimulationLoop(argc,argv,winx,winy,&fn);

  dCloseODE();
}
//-------------------------------------------------------------------------------------------

void Stop()
{
  dsStop();
}
//-------------------------------------------------------------------------------------------


}  // end of ode_x
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
}  // end of trick
//-------------------------------------------------------------------------------------------
