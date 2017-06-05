#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <iostream>

#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif


int main(int argc, char* argv[])
{
  // set model path
  std::string file_path;
  if (argc > 1){
    file_path = argv[1];
  } else {
    std::cout << "usage: rosrun hrpsys_samples testModelLoader [modelpath]" << std::endl;
    return 0;
  }
  // set variables
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  hrp::BodyPtr m_robot(new hrp::Body());
  // load models
  if (!loadBodyFromModelLoader(m_robot, file_path.c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext()))) {
    std::cerr << "Failed to load model"  << std::endl;
    std::cerr << "Please check if openhrp-model-loader is running"  << std::endl;
  }
  // set joint angle
  double reset_pose_angles[] = {-0.004457, -21.692900, -0.012020, 47.672300, -25.930000, 0.014025,
                                17.835600, -9.137590, -6.611880, -36.456000, 0.000000, 0.000000, 0.000000,
                                -0.004457, -21.692900, -0.012020, 47.672300, -25.930000, 0.014025,
                                17.835600, 9.137590, 6.611880, -36.456000, 0.000000, 0.000000, 0.000000,
                                0.000000, 0.000000, 0.000000};
  std::cout << "---------" << std::endl;
  std::cout << "< link list >" << std::endl;
  for (size_t i = 0; i < m_robot->numJoints(); ++i) {
    m_robot->joint(i)->q += deg2rad(reset_pose_angles[i]);
    std::cout << m_robot->link(i)->name << std::endl;
  }
  m_robot->calcForwardKinematics();
  // calc Jacobian
  hrp::dmatrix J;
  hrp::JointPath* jp = new hrp::JointPath(m_robot->link(0), m_robot->link(5));
  jp->calcJacobian(J);
  std::cout << "--------" << std::endl;
  std::cout << "< Jacobian >" << std::endl;
  std::cout << J << std::endl;
}
