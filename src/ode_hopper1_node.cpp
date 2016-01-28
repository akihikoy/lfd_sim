//-------------------------------------------------------------------------------------------
/*! \file    ode_hopper1_node.cpp
    \brief   Single-leg hopping robot simulator using ODE.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Dec.28, 2015
*/
//-------------------------------------------------------------------------------------------
#include "lfd_sim/ode_hopper1.h"
#include "lfd_sim/ODEConfigHp1.h"
#include "lfd_sim/ODESensorHp1.h"
#include "lfd_sim/ODEControlHp1.h"
#include "lfd_sim/ODEViz.h"
#include "lfd_sim/ODEVizPrimitive.h"
#include "lfd_sim/ODEGetConfigHp1.h"
#include "lfd_sim/ODESetConfigHp1.h"
//-------------------------------------------------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
//-------------------------------------------------------------------------------------------
namespace trick
{

ros::Publisher *PSensorsPub(NULL), *PKeyEventPub(NULL);

std::vector<lfd_sim::ODEVizPrimitive> VizObjs;

void OnSensingCallback(const ode_x::TSensorsHp1 &sensors)
{
  // Copy sensors to sensor message
  lfd_sim::ODESensorHp1 msg;

  #define COPY(cid)  msg.cid= sensors.cid;
  COPY( JointAngles  )
  COPY( LinkX        )
  COPY( Forces       )
  COPY( Masses       )
  COPY( Collisions   )
  COPY( Time         )
  #undef COPY

  PSensorsPub->publish(msg);
}
//-------------------------------------------------------------------------------------------

void OnDrawCallback()
{
  dReal x[7]={0.0,0.0,0.0, 1.0,0.0,0.0,0.0};
  dReal sides[4]={0.0,0.0,0.0,0.0};
  dMatrix3 R;
  dVector3 p;
  for(std::vector<lfd_sim::ODEVizPrimitive>::const_iterator
      itr(VizObjs.begin()),itr_e(VizObjs.end()); itr!=itr_e; ++itr)
  {
    dsSetColorAlpha(itr->color.r,itr->color.g,itr->color.b,itr->color.a);
    GPoseToX(itr->pose, x);
    dReal q[4]= {x[6],x[3],x[4],x[5]};
    switch(itr->type)
    {
    case lfd_sim::ODEVizPrimitive::LINE:
      p[0]= x[0]+itr->param[0];
      p[1]= x[1]+itr->param[1];
      p[2]= x[2]+itr->param[2];
      dsDrawLine(x, p);
      break;
    case lfd_sim::ODEVizPrimitive::SPHERE:
      dRfromQ(R,q);
      dsDrawSphere(x, R, /*rad=*/itr->param[0]);
      break;
    case lfd_sim::ODEVizPrimitive::CYLINDER:
      dRfromQ(R,q);
      dsDrawCylinder(x, R, /*len=*/itr->param[1], /*rad=*/itr->param[0]);
      break;
    case lfd_sim::ODEVizPrimitive::CUBE:
      dRfromQ(R,q);
      sides[0]= itr->param[0];
      sides[1]= itr->param[1];
      sides[2]= itr->param[2];
      dsDrawBox(x, R, sides);
      break;
    default:
      std::cerr<<"Unknown type:"<<itr->type<<std::endl;
      return;
    }
  }
}
//-------------------------------------------------------------------------------------------

void OnStepCallback(const double &time, const double &time_step)
{
  if(ode_x::Running)
    std::cerr<<"@"<<time<<std::endl;
  else
    usleep(500*1000);
  ros::spinOnce();
  if(!ros::ok())  ode_x::Stop();
}
//-------------------------------------------------------------------------------------------

void OnKeyEventCallback(int command)
{
  std_msgs::Int32 keyevent_msg;
  keyevent_msg.data= command;
  PKeyEventPub->publish(keyevent_msg);
}
//-------------------------------------------------------------------------------------------

void ControlCallback(const lfd_sim::ODEControlHp1 &msg)
{
  #define COPY(cid)  ode_x::target::cid= msg.cid;
  COPY( Angles  )
  COPY( Vel     )
  COPY( Torque  )
  #undef COPY
}
//-------------------------------------------------------------------------------------------

void ODEVizCallback(const lfd_sim::ODEViz &msg)
{
  VizObjs= msg.objects;
}
//-------------------------------------------------------------------------------------------

bool GetConfig(lfd_sim::ODEGetConfigHp1::Request &req, lfd_sim::ODEGetConfigHp1::Response &res)
{
  using namespace ode_x;
  lfd_sim::ODEConfigHp1 &msg(res.config);

  #define COPY(cid)  msg.cid= cid;
  COPY( VizJointLen    )
  COPY( VizJointRad    )
  COPY( FSThick        )
  COPY( FSSize         )
  COPY( BaseLenX       )
  COPY( BaseLenY       )
  COPY( BaseLenZ       )
  COPY( LegRad         )
  COPY( LegLen1        )
  COPY( LegLen2        )
  COPY( EyeCylR        )
  COPY( EyeCylL        )
  COPY( FootRad        )
  COPY( EyeRad         )
  COPY( BaseDensity    )
  COPY( OtherDensity   )
  COPY( TerrainMode    )
  COPY( MaxContacts    )
  COPY( TimeStep       )
  COPY( Gravity        )
  COPY( EnableKeyEvent )
  COPY( HingeFMax      )
  COPY( ControlMode    )
  #undef COPY

  return true;
}
//-------------------------------------------------------------------------------------------

bool ResetSim2(lfd_sim::ODESetConfigHp1::Request &req, lfd_sim::ODESetConfigHp1::Response &res)
{
  using namespace ode_x;
  std::cerr<<"Resetting simulator..."<<std::endl;
  const lfd_sim::ODEConfigHp1 &msg(req.config);

  #define COPY(cid)  cid= msg.cid;
  COPY( VizJointLen    )
  COPY( VizJointRad    )
  COPY( FSThick        )
  COPY( FSSize         )
  COPY( BaseLenX       )
  COPY( BaseLenY       )
  COPY( BaseLenZ       )
  COPY( LegRad         )
  COPY( LegLen1        )
  COPY( LegLen2        )
  COPY( EyeCylR        )
  COPY( EyeCylL        )
  COPY( FootRad        )
  COPY( EyeRad         )
  COPY( BaseDensity    )
  COPY( OtherDensity   )
  COPY( TerrainMode    )
  COPY( MaxContacts    )
  COPY( TimeStep       )
  COPY( Gravity        )
  COPY( EnableKeyEvent )
  COPY( HingeFMax      )
  COPY( ControlMode    )
  #undef COPY

  ode_x::Reset();
  std::cerr<<"done."<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

bool ResetSim(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resetting simulator..."<<std::endl;
  ode_x::Reset();
  std::cerr<<"done."<<std::endl;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Simulator paused..."<<std::endl;
  ode_x::Running= false;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Simulator resumed..."<<std::endl;
  ode_x::Running= true;
  return true;
}
//-------------------------------------------------------------------------------------------

}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "ode_hopper1");
  ros::NodeHandle node("~");

  std::string texture_path;
  int winx(500), winy(400);
  node.param("texture_path",texture_path,std::string("config/textures"));
  node.param("winx",winx,winx);
  node.param("winy",winy,winy);

  ros::Publisher sensors_pub= node.advertise<lfd_sim::ODESensorHp1>("sensors", 1);
  PSensorsPub= &sensors_pub;
  ros::Publisher keyevent_pub= node.advertise<std_msgs::Int32>("keyevent", 1);
  PKeyEventPub= &keyevent_pub;

  ros::Subscriber sub_control= node.subscribe("control", 1, &ControlCallback);
  ros::Subscriber sub_viz= node.subscribe("viz", 1, &ODEVizCallback);
  ros::ServiceServer srv_get_config= node.advertiseService("get_config", &GetConfig);
  ros::ServiceServer srv_reset= node.advertiseService("reset", &ResetSim);
  ros::ServiceServer srv_reset2= node.advertiseService("reset2", &ResetSim2);
  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);

  ode_x::SensingCallback= &OnSensingCallback;
  ode_x::DrawCallback= &OnDrawCallback;
  ode_x::StepCallback= &OnStepCallback;
  ode_x::KeyEventCallback= &OnKeyEventCallback;
  ode_x::Run(argc, argv, texture_path.c_str(), winx, winy);

  return 0;
}
//-------------------------------------------------------------------------------------------
