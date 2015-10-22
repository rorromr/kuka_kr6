/* Basado en https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/src/basic_controls.cpp
 */


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
InteractiveMarker kuka_marker;
tf::Transform marker_tf;
geometry_msgs::PoseStamped markerMsg;
ros::Publisher markerPub;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  std::ostringstream mouse_point_ss;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  tf::Vector3 origin;
  tf::Quaternion rotation;
  switch (feedback->event_type){
  	  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:

  		  /*markerMsg.header=feedback->header;
  		  markerMsg.header.stamp=ros::Time::now();
  		  markerMsg.pose=feedback->pose;
  		  markerPub.publish(markerMsg);*/


		  ROS_INFO_STREAM( s.str() << ": pose changed"
			  << "\nposition = "
			  << feedback->pose.position.x
			  << ", " << feedback->pose.position.y
			  << ", " << feedback->pose.position.z
			  << "\norientation = "
			  << feedback->pose.orientation.w
			  << ", " << feedback->pose.orientation.x
			  << ", " << feedback->pose.orientation.y
			  << ", " << feedback->pose.orientation.z
			  << "\nframe: " << feedback->header.frame_id
			  << " time: " << feedback->header.stamp.sec << "sec, "
			  << feedback->header.stamp.nsec << " nsec" );

		  /*origin=tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
		  rotation=tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);
		  marker_tf.setOrigin(origin);
		  marker_tf.setRotation(rotation);*/
  		 // ROS_INFO("ACTUALIZA");
		  break;

  	  /* MENU
  	  case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
  		  origin=marker_init_tf.getOrigin();
  		  rotation=marker_init_tf.getRotation();
		  ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
		  kuka_marker.pose.position.x=origin.getX();
		  kuka_marker.pose.position.y=origin.getY();
		  kuka_marker.pose.position.z=origin.getZ();
		  tf::quaternionTFToMsg(rotation, kuka_marker.pose.orientation);
		  break;
		*/
  }

  server->applyChanges();
}

/*
 *
 *
 */

visualization_msgs::InteractiveMarker make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Pose& pose, bool show_6dof , std::string frame_id)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  tf::poseTFToMsg(pose, int_marker.pose);
  int_marker.scale = 0.2;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  return int_marker;
}

visualization_msgs::InteractiveMarker makeRotateMarker(int x, int y, int z, std::string marker_name, std::string frame_id, float scale)
{
  InteractiveMarker int_marker;

  // Pose inicial
  int_marker.header.frame_id = frame_id; // Frame
  int_marker.pose.orientation.w=1;
  int_marker.pose.orientation.x=0;
  int_marker.pose.orientation.y=0;
  int_marker.pose.orientation.z=0;
  int_marker.pose.position.x=0;
  int_marker.pose.position.y=0;
  int_marker.pose.position.z=0;

  // Parametros
  int_marker.scale = scale;
  int_marker.name = marker_name;
  int_marker.description = "joint angle for " + frame_id;

  InteractiveMarkerControl control;
  // Orientacion
  control.orientation.w = 1;
  control.orientation.x = x;
  control.orientation.y = z;
  control.orientation.z = y;
  control.name = "rotate";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  //ROS_INFO("w=%f  x=%f  y=%f  z=%f",control.orientation.w,control.orientation.x,control.orientation.y,control.orientation.z);

  return int_marker;
}
// %EndTag(6DOF)%


int main(int argc, char** argv)
{
	ros::init(argc, argv, "kuka_pose_marker");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	// Parametro rate
	int rate;
	nh.param("rate", rate, 30);
	ros::Rate loop_rate(rate);
	ROS_INFO("Marker tf: %d Hz", rate);

	// Parametro frame_id
	std::string frame_id;
	nh.param<std::string>("frame_id", frame_id, "/map");
	ROS_INFO("Marker tf frame_id: %s", frame_id.c_str());

	// Parametro marker_frame
	std::string marker_frame;
	nh.param<std::string>("marker_frame", marker_frame, "/marker_frame");
	ROS_INFO("Marker tf frame: %s", marker_frame.c_str());


	// Parametro init_pose
	std::vector<float> init_pose (6,0.0);
	if (!nh.getParam("init_pose", init_pose)){
		ROS_INFO("Marker default init pose (0 0 0).");
	}
	// Menejo de posición inicial
	tf::Pose pose;
	tf::Quaternion q;
	tf::Vector3 v=tf::Vector3(init_pose[0], init_pose[1], init_pose[2]);
	q.setRPY(init_pose[3], init_pose[4], init_pose[5]);
	// Posición inicial
	pose.setOrigin(v);
	pose.setRotation(q);
	// TF inicial
	marker_tf.setOrigin(v);
	marker_tf.setRotation(q);

	// Definir transformación
	static tf::TransformBroadcaster br;

	// Iniciar servidor Marker
	server.reset( new interactive_markers::InteractiveMarkerServer("kuka_controls","",false) );
	ros::Duration(0.1).sleep();

	// Elementos del menu
	//menu_handler.insert( "Go Home", &processFeedback );

	// Crear 6DOF Marker
	//kuka_marker = make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, pose, true, frame_id);

	//server->insert(kuka_marker);
	//server->setCallback(kuka_marker.name, &processFeedback);
	//menu_handler.apply( *server, kuka_marker.name );

	// Crear Marker para joints
	InteractiveMarker a1_marker = makeRotateMarker(0, 0, 1,"a1","link_1",0.55);
	server->insert(a1_marker);
	server->setCallback(a1_marker.name, &processFeedback);

	InteractiveMarker a2_marker = makeRotateMarker(0, 1, 0,"a2","link_2",0.35);
	server->insert(a2_marker);

	InteractiveMarker a3_marker = makeRotateMarker(0, 1, 0,"a3","link_3",0.25);
	server->insert(a3_marker);


	server->applyChanges();

	// Marker OK
	ROS_INFO("KUKA IK Marker [OK]");

	// Publisher para posición
	//markerPub = n.advertise<geometry_msgs::PoseStamped>("marker_pose", 1000);

	ros::spin();

/*	while (ros::ok()) {
		// Publicar tf Marker
		br.sendTransform(tf::StampedTransform(marker_tf, ros::Time::now(), frame_id, marker_frame));
		ros::spinOnce();
		loop_rate.sleep();
	}*/

	server.reset();
}
