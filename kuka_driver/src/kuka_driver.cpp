#include <iostream>
#include <queue>
// Boost Thread
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
// KUKA services
#include <kuka_driver/SetVelocity.h>
#include <kuka_driver/Stop.h>
#include <kuka_driver/PTP.h>
#include <kuka_driver/Home.h>
// KUKA LIB
#include <kuka_driver/kukalib.h>


namespace kuka_driver
{
// Constatnts
const float PI = 3.14159265358979f, rad2deg = 180.0 / 3.14159265358979,
  deg2rad = 3.14159265358979 / 180.0;

// Enum for msg types
enum Option { STOP, PTP, LIN, VEL}; //for CASE

// Store msg info
struct data
{
  float ptp_goal[6]; // PTP joint data
  int vel; // Desired velocity
};
typedef struct data Data;

class KukaRequest
{
  public:
    KukaRequest(){
      // Set priority
      priority_ = 0;
    }
    
    KukaRequest(int priority)
    {
      // Set priority
      this->priority_ = priority;
    }
    
    KukaRequest(int priority, Option opt)
    {
      // Set priority
      this->priority_ = priority;
      this->opt_ = opt;
    }

    /*
    KukaRequest(KukaRequest& req){
      // Set the pointer
      priority_ = req.getPriority();
    }*/

    ~KukaRequest(){}

    int getPriority() const {return priority_;}

    bool operator<(const KukaRequest& req) const
    {
      return priority_ < req.getPriority();
    }

  public:
    Option opt_;
    int priority_;
    Data data_;
};

class KukaRequestServer
{
  public:
    KukaRequestServer(bool enable_debug = false):
      nh_("~"),
      name_("kuka_driver")
    {
      // Joint state publisher
      joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
      // Velocity server
      vel_server_ = nh_.advertiseService("set_vel", &KukaRequestServer::setVel, this);
      stop_server_ = nh_.advertiseService("stop", &KukaRequestServer::stop, this);
      home_server_ = nh_.advertiseService("home", &KukaRequestServer::home, this);
      ptp_server_ = nh_.advertiseService("ptp", &KukaRequestServer::ptp, this);
      
      // KUKA initialisation @TODO server name parameter
      kuka_.beginConnection("kuka");

      // Command thread
      command_thread_ = boost::thread(boost::bind(&KukaRequestServer::sendCommad, this));

      // Debug flag, set logger level to DEBUG
      if(enable_debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
         ros::console::notifyLoggerLevelsChanged();

      // Joint offset
      for (unsigned int i = 0; i < 6; ++i)
      {
        offset_[i] = 0.0;
        position_d_[i] = 0.0;
      }
      offset_[1] = -90.0;
      offset_[2] = 90.0;

      // Joint update


      // Joint state msg
      joint_state_.position.resize(6,0.0);
      joint_state_.velocity.resize(6,0.0);
      joint_state_.effort.resize(6,0.0);
      joint_state_.name.resize(6);
      // Joint name
      joint_state_.name[0]="a1";
      joint_state_.name[1]="a2";
      joint_state_.name[2]="a3";
      joint_state_.name[3]="a4";
      joint_state_.name[4]="a5";
      joint_state_.name[5]="a6";

      /*
      TODO JOINT LIMITS

      A2 -140 -15
      A5 -95 85

      */

    };

    ~KukaRequestServer(){
      nh_.shutdown();
      command_thread_.join();
    };

    bool addRequest(KukaRequest& req){
      boost::mutex::scoped_lock lock(queue_lock_);
      // Add request to queue
      queue_.push(req);
      return true;
    }

    KukaRequest getRequest(){
      boost::mutex::scoped_lock lock(queue_lock_);
      // Return queue top
      return queue_.top();
    }

    void pop(){
      boost::mutex::scoped_lock lock(queue_lock_);
      // Remove top element
      queue_.pop();
    }

    unsigned int getRequestNum() const {return queue_.size();}

    bool empty() const {return queue_.empty();}

    bool setVel(SetVelocity::Request &req, SetVelocity::Response &res){
      // Create
      // @TODO Saturacion
      KukaRequest kuka_req( 3, VEL); //(priority, SRV)
      kuka_req.data_.vel = req.desired_vel;
      addRequest(kuka_req);
      res.result = true;
      ROS_DEBUG_NAMED(name_, "Set velocity command with: %i", kuka_req.data_.vel);
      return true;
    }

    bool stop(Stop::Request &req, Stop::Response &res){
      KukaRequest kuka_req( 10, STOP); //(priority, SRV)
      addRequest(kuka_req);
      ROS_WARN_NAMED(name_, "Stop command received.");
      res.result = true;
      return true;
    }

    bool home(Home::Request &req, Home::Response &res){
      KukaRequest kuka_req( 2, PTP); //(priority, SRV)
      // Add joint command in rads and add offset for ROS model
      for(unsigned int i=0; i<6; i++) kuka_req.data_.ptp_goal[i]=0.0*rad2deg+offset_[i];
      addRequest(kuka_req);
      ROS_DEBUG_NAMED(name_, "HOME command received.");
      res.result = true;
      return true;
    }  

    bool ptp(PTP::Request &req, PTP::Response &res){
      KukaRequest kuka_req( 2, PTP); //(priority, SRV)
      // Add joint command in rads and add offset for ROS model
      for(unsigned int i=0; i<6; i++) kuka_req.data_.ptp_goal[i]=req.position[i]*rad2deg+offset_[i];
      addRequest(kuka_req);
      ROS_DEBUG_NAMED(name_, "PTP command received.");
      res.result = true;
      return true;
    }

    void sendCommad(){
      ros::Duration sleep_time(0.1);
      bool error = false;
      unsigned int num_req;
      while(nh_.ok() && ros::ok() && !error){
        num_req = getRequestNum();
        if (num_req > 0){
          KukaRequest req = getRequest();
          pop(); // TODO
          // Check option
          switch (req.opt_){
            case STOP:
              // Send Stop
              error = kuka_.stop();
              ROS_WARN_NAMED(name_, "Sending stop request to KUKA robot.");
              break;
            case PTP:
              // Send PTP command
              kuka_.setJointPosition(req.data_.ptp_goal);
              ROS_DEBUG_NAMED(name_, "Sending PTP request to KUKA robot.");
              break;
            case VEL:
              // Send PTP command
              kuka_.setOverrideSpeed(req.data_.vel);
              ROS_DEBUG_NAMED(name_, "Sending velocity override request to KUKA robot.");
              break;
            // case HOME:
            //   // Send Home 
            //   for(unsigned int i=0; i<6; i++) req.data_.ptp_goal[i]=0.0*rad2deg+offset_[i];
            //   kuka_.setJointPosition(req.data_.ptp_goal); 
            //   ROS_DEBUG_NAMED(name_, "Sending HOME request to KUKA robot.");
            //   break;            
            default:
              ROS_ERROR_NAMED(name_, "Undefined option type.");
          }
          sleep_time = ros::Duration(0.1);
        } 
        // Update joint state
        else if (num_req == 0) {
          ROS_DEBUG_NAMED(name_, "Update request to KUKA robot.");
          // Call KUKA update
          kuka_.update();
          // Reset delta position and norm values
          double norm = 0.0;
          for (unsigned int i = 0; i < 6; ++i) position_d_[i] = joint_state_.position[i];
          
          // Get joint position
          kuka_.getJointPosition(joint_state_.position);
          
          // Offset and rad convertion
          for (unsigned int i = 0; i < 6; ++i)
          {
            joint_state_.position[i] = (joint_state_.position[i] - offset_[i])*deg2rad;
            position_d_[i] -= joint_state_.position[i];
            norm += position_d_[i]*position_d_[i];
          }
          //norm = sqrt(norm);
          if ( norm < 0.01 ) sleep_time = ros::Duration(1);
          ROS_DEBUG_NAMED(name_, "Norma: %.2f", norm);

          // Publish joint state
          joint_state_.header.stamp = ros::Time::now();
          joint_state_.header.seq++;
          joint_state_pub_.publish(joint_state_);

        }
        ROS_DEBUG_THROTTLE_NAMED(0.2, name_, "Current number of req %i", getRequestNum());
        sleep_time.sleep();
      }
    }

    
  private:
    // Request priority queue
    std::priority_queue<KukaRequest> queue_;
    // Mutex to lock access to priotity queue
    boost::mutex queue_lock_;
    boost::thread command_thread_;
    // ROS Nodehandle
    ros::NodeHandle nh_;
    // Joint state publisher
    ros::Publisher joint_state_pub_;
    // Services
    ros::ServiceServer vel_server_;  // Velocity config
    ros::ServiceServer stop_server_;  // Stop
    ros::ServiceServer ptp_server_;  // ptp : Point To Point
    ros::ServiceServer home_server_;  // home
    // KUKA
    kukalib::Kuka kuka_;
    // Joint offset
    float offset_[6];
    // Joint position delta
    float position_d_[6];
    // Name for log messages
    const std::string name_;
    // Base joint state msg
    sensor_msgs::JointState joint_state_;
};

} //kuka_driver namespace


int main(int argc, char **argv)
{
  using namespace kuka_driver;

  ros::init(argc, argv, "kuka_driver");
  
  KukaRequestServer server(true);
  
  ros::spin();

  std::cout<<"server contains " << server.getRequestNum() << " elements.\n";

  while (!server.empty()) {
    std::cout << server.getRequest().data_.vel << std::endl;  //priority Height object in the priority queue
    server.pop();                                               //remove the highest priority element
  }
}