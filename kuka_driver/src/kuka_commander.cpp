//19 Mayo 2015

// KUKA commander
#include <kuka_driver/kuka_commander.h>

namespace kuka_driver
{

    KukaCommander::KukaCommander(std::string ns, bool enable_debug):
      nh_(),
      name_("kuka_commander"),
      ns_(ns)
    {
      ROS_INFO_NAMED(name_, "Init Kuka Commander");


      // Clientes
      vel_client_ = nh_.serviceClient<kuka_driver::SetVelocity>(ns+"/set_vel");
      ptp_client_ = nh_.serviceClient<kuka_driver::PTP>(ns+"/ptp");
      home_client_ = nh_.serviceClient<kuka_driver::Home>(ns+"/home");
      stop_client_ = nh_.serviceClient<kuka_driver::PTP>(ns+"/stop");
      
    }

    bool KukaCommander::set_velocity(int desired_vel)
    {
      SetVelocity srv;
      srv.request.desired_vel = desired_vel;
      if (vel_client_.call(srv))
      {
        ROS_INFO_NAMED(name_, "Setting velocity to %i", desired_vel);
        return srv.response.result;
      }
      else
      {
        ROS_ERROR_NAMED(name_, "Failed to call service set_velocity");
      }
      return false;
    }
    

    bool KukaCommander::ptp(const std::vector<double>& joint_position)
    {
      if (joint_position.size()>6)
      {
        ROS_ERROR_NAMED(name_, "Wrong number of joint values");
        return false;
      }
      PTP srv;
      for (unsigned int i=0; i<6 ; i++)
      {
        srv.request.position[i] = joint_position[i];
      }
      
      if (ptp_client_.call(srv))
      {
        ROS_INFO_NAMED(name_, "Sending PTP request");
        // DEBUG CON VALOR DE JOINTS
        return srv.response.result;
      }
      else
      {
        ROS_ERROR_NAMED(name_, "Failed to call service ptp");
      }
      return false;
    }

    bool KukaCommander::home()
    {
      Home srv;
      if (home_client_.call(srv))
      {
        ROS_INFO_NAMED(name_, "Sending Home request");
        return srv.response.result;
      }
      else
      {
        ROS_ERROR_NAMED(name_, "Failed to call service home");
      }
      return false;
    }

    bool KukaCommander::stop()
    {
      Stop srv;
      ROS_INFO_NAMED(name_, "Sending Stop request");
      if (stop_client_.call(srv))
      {
        return srv.response.result;
      }
      else
      {
        ROS_ERROR_NAMED(name_, "Failed to call service home");
      }
      return false;
    }

} //kuka_driver namespace


int main(int argc, char **argv){
  return 0;
}