// ROS
#include <ros/ros.h>
#include <ros/console.h>
// KUKA services
#include <kuka_driver/SetVelocity.h>
#include <kuka_driver/Stop.h>
#include <kuka_driver/PTP.h>
#include <kuka_driver/Home.h>


namespace kuka_driver
{

class KukaCommander
{
    
  private:
    ros::NodeHandle nh_;
    const std::string name_;
    std::string ns_;

    ros::ServiceClient
      vel_client_,
      ptp_client_,
      home_client_,
      stop_client_;

  public:
    KukaCommander(std::string ns = "",bool enable_debug = false);

    bool set_velocity(int desired_vel);
    
    bool ptp(const std::vector<double>& joint_position);

    bool home();
   
    bool stop();

};

} //kuka_driver namespace