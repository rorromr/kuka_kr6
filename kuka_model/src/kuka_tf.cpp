#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>



int main(int argc, char** argv) {
	ros::init(argc, argv, "kuka_tf");
	ros::NodeHandle node;
	ros::NodeHandle nh("~");

	// Parametro rate
	int rate;
	nh.param("rate", rate, 30);
	ros::Rate loop_rate(rate);
	ROS_INFO("KUKA tf para base en: %d Hz", rate);

  // Parametro source_frameid
  std::string source_frameid;
  nh.param<std::string>("source_frameid", source_frameid, "/world");
  ROS_INFO("source_frameid: %s", source_frameid.c_str());

  // Parametro to_frame
  std::string shadow_target_frameid;
  nh.param<std::string>("shadow_target_frameid", shadow_target_frameid, "/shadow/link_6");
  ROS_INFO("shadow_target_frameid: %s", shadow_target_frameid.c_str());

  // Parametro to_frame
  std::string real_target_frameid;
  nh.param<std::string>("real_target_frameid", real_target_frameid, "/real/link6");
  ROS_INFO("real_target_frameid: %s", real_target_frameid.c_str());

	ros::Publisher shadowEffectorPub = node.advertise<geometry_msgs::PoseStamped>("shadow_effector_pose", 100);
  ros::Publisher shadowEffectorBodkinPub = node.advertise<geometry_msgs::PoseStamped>("shadow_effector_pose_bodkin", 100);
  ros::Publisher realEffectorPub = node.advertise<geometry_msgs::PoseStamped>("real_effector_pose", 100);
  ros::Publisher realEffectorBodkinPub = node.advertise<geometry_msgs::PoseStamped>("real_effector_pose_bodkin", 100);

	geometry_msgs::PoseStamped shadow_pose_msg,shadow_pose_bodkin_msg,real_pose_msg,real_pose_bodkin_msg;
	tf::Vector3 posicion, posicion_bodkin, posicion_real_bodkin;
	tf::Quaternion rotacion, rotacion_bodkin, rotacion_real_bodkin;

	// Definir transformación
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Vector3 pos(0.0, 0.0, 0.0);
	transform.setOrigin( pos);
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);

  // Definir transformación BODKIN
  static tf::TransformBroadcaster brBOD;
  tf::Transform transformBOD;
  tf::Vector3 posBOD(0.31, 0.0, 0.0);
  transformBOD.setOrigin(posBOD);
  tf::Quaternion qBOD;
  qBOD.setRPY(0, 0, 0);
  transformBOD.setRotation(qBOD);

  // Definir transformación KINECT
  static tf::TransformBroadcaster brKinect;
  tf::Transform transformKinect;
  tf::Vector3 posKinect(-0.1, -0.1, -0.1); //TODO:  DYNAMIC RECONFIGURE
  transformKinect.setOrigin(posKinect);
  tf::Quaternion qKinect;
  qKinect.setRPY(0, 0, 0);  //TODO:  DYNAMIC RECONFIGURE
  transformKinect.setRotation(qKinect);

	tf::StampedTransform shadow_efector_transform, shadow_efector_transform_bodkin, real_efector_transform, real_efector_transform_bodkin;
	tf::TransformListener listener;
	ros::Duration(1.0).sleep();

	while (ros::ok()) {
    // Publicar publicaciones para KUKA Real y KUKA Shadow en el mismo origen
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "real/base"));
    brBOD.sendTransform(tf::StampedTransform(transformBOD, ros::Time::now(), "/real/link6", "/real/link6_bodkin"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "shadow/base"));
    brBOD.sendTransform(tf::StampedTransform(transformBOD, ros::Time::now(), "/shadow/link_6", "/shadow/link_6_bodkin"));
    //brKinect.sendTransform(tf::StampedTransform(transformKinect,ros::Time::now(), "/real/kinect_base_link", "/real/kinect_dynamic"));

    //asdasd

    try{
      // Buscar tf shadow
      listener.lookupTransform(source_frameid, shadow_target_frameid,ros::Time(0), shadow_efector_transform);
      // Buscar tf shadow bodkin
      listener.lookupTransform(source_frameid, "/shadow/link_6_bodkin",ros::Time(0), shadow_efector_transform_bodkin);
      // Buscar tf real
      listener.lookupTransform(source_frameid, real_target_frameid, ros::Time(0), real_efector_transform);
      // Buscar tf real bodkin
      listener.lookupTransform(source_frameid, "/real/link6_bodkin",ros::Time(0), real_efector_transform_bodkin);
      
      // Msg Shadow
      // Header
      shadow_pose_msg.header.frame_id=shadow_efector_transform.frame_id_;
      shadow_pose_msg.header.seq++;
      shadow_pose_msg.header.stamp=ros::Time::now();
      // Posicion
      posicion=shadow_efector_transform.getOrigin();
      shadow_pose_msg.pose.position.x=posicion.getX();
      shadow_pose_msg.pose.position.y=posicion.getY();
      shadow_pose_msg.pose.position.z=posicion.getZ();
      // Rotacion
      rotacion=shadow_efector_transform.getRotation();
      shadow_pose_msg.pose.orientation.x=rotacion.getX();
      shadow_pose_msg.pose.orientation.y=rotacion.getY();
      shadow_pose_msg.pose.orientation.z=rotacion.getZ();
      shadow_pose_msg.pose.orientation.w=rotacion.getW();

      // Msg Real
      // Header
      real_pose_msg.header.frame_id=real_efector_transform.frame_id_;
      real_pose_msg.header.seq++;
      real_pose_msg.header.stamp=ros::Time::now();
      // Posicion
      posicion=real_efector_transform.getOrigin();
      real_pose_msg.pose.position.x=posicion.getX();
      real_pose_msg.pose.position.y=posicion.getY();
      real_pose_msg.pose.position.z=posicion.getZ();
      // Rotacion
      rotacion=real_efector_transform.getRotation();
      real_pose_msg.pose.orientation.x=rotacion.getX();
      real_pose_msg.pose.orientation.y=rotacion.getY();
      real_pose_msg.pose.orientation.z=rotacion.getZ();
      real_pose_msg.pose.orientation.w=rotacion.getW();

      // Msg Shadow Bodkin
      // Header
      shadow_pose_bodkin_msg.header.frame_id=shadow_efector_transform_bodkin.frame_id_;
      shadow_pose_bodkin_msg.header.seq++;
      shadow_pose_bodkin_msg.header.stamp=ros::Time::now();
      // Posicion
      posicion_bodkin=shadow_efector_transform_bodkin.getOrigin();
      shadow_pose_bodkin_msg.pose.position.x=posicion_bodkin.getX();
      shadow_pose_bodkin_msg.pose.position.y=posicion_bodkin.getY();
      shadow_pose_bodkin_msg.pose.position.z=posicion_bodkin.getZ();
      // Rotacion
      rotacion_bodkin=shadow_efector_transform_bodkin.getRotation();
      shadow_pose_bodkin_msg.pose.orientation.x=rotacion_bodkin.getX();
      shadow_pose_bodkin_msg.pose.orientation.y=rotacion_bodkin.getY();
      shadow_pose_bodkin_msg.pose.orientation.z=rotacion_bodkin.getZ();
      shadow_pose_bodkin_msg.pose.orientation.w=rotacion_bodkin.getW();

      // Msg Real Bodkin
      // Header
      real_pose_bodkin_msg.header.frame_id=real_efector_transform_bodkin.frame_id_;
      real_pose_bodkin_msg.header.seq++;
      real_pose_bodkin_msg.header.stamp=ros::Time::now();
      // Posicion
      posicion_real_bodkin=real_efector_transform_bodkin.getOrigin();
      real_pose_bodkin_msg.pose.position.x=posicion_real_bodkin.getX();
      real_pose_bodkin_msg.pose.position.y=posicion_real_bodkin.getY();
      real_pose_bodkin_msg.pose.position.z=posicion_real_bodkin.getZ();
      // Rotacion
      rotacion_real_bodkin=real_efector_transform_bodkin.getRotation();
      real_pose_bodkin_msg.pose.orientation.x=rotacion_real_bodkin.getX();
      real_pose_bodkin_msg.pose.orientation.y=rotacion_real_bodkin.getY();
      real_pose_bodkin_msg.pose.orientation.z=rotacion_real_bodkin.getZ();
      real_pose_bodkin_msg.pose.orientation.w=rotacion_real_bodkin.getW();



      shadowEffectorPub.publish(shadow_pose_msg);
      shadowEffectorBodkinPub.publish(shadow_pose_bodkin_msg);
      realEffectorPub.publish(real_pose_msg);
      realEffectorBodkinPub.publish(real_pose_bodkin_msg);

    }
    catch (tf::TransformException &ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    ros::spinOnce();
    loop_rate.sleep();
	}
	return 0;
};
