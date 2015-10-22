//Basado en el ejemplo de kinematic_model_tutorial para el robot pr2.

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

void kukaEfectorCallback(const geometry_msgs::Pose & msg);
geometry_msgs::Pose goalPose;

int main(int argc, char **argv)
{
	// Iniciar nodo IK
	ros::init (argc, argv, "kuka_ik");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	// Parametro rate
	int rate;
	nh.param("rate", rate, 5);
	ros::Rate loop_rate(rate);
	ROS_INFO("IK Solve: %d Hz", rate);

	// Parametro origin
	std::string origin;
	nh.param<std::string>("origin", origin, "world");

	// Cargar robot KUKA
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	// Frame por defecto base
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	// Grupo de movimiento del robot
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
	// Obtener nombres de joints
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	// Posicion actual de joints
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

	// Publisher para posiciones KUKA Shadow
	ros::Publisher kukaShadowPub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

	// Suscriber para posiciones objetivo para KUKA Shadow
	ros::Subscriber kukaShadowSub = n.subscribe("efector_pose", 50, kukaEfectorCallback);


	// Mensaje para KUKA Shadow
	sensor_msgs::JointState shadowJointMsg;
	shadowJointMsg.name.resize(6);
	shadowJointMsg.position.resize(6);
	shadowJointMsg.velocity.resize(6);
	for(std::size_t i=0; i < joint_names.size(); ++i){
		shadowJointMsg.name[i] = joint_names[i].c_str();
		shadowJointMsg.position[i] = joint_values[i];
	}

	// Mensaje
	while (ros::ok()) {

		// IK
		kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

		bool found_ik = kinematic_state->setFromIK(joint_model_group, goalPose, 5, 0.01);

		// Now, we can print out the IK solution (if found):
		if (found_ik) {
			// Actualizar joint_values
			kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

			// Publicar mensaje para KUKA Shadow
			shadowJointMsg.header.stamp = ros::Time::now();
			for(std::size_t i=0; i < joint_names.size(); ++i){
				shadowJointMsg.name[i] = joint_names[i].c_str();
				shadowJointMsg.position[i] = joint_values[i];
			}
			kukaShadowPub.publish(shadowJointMsg);
			/*
			for(std::size_t i=0; i < joint_names.size(); ++i){
				ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
			}
			*/
		}
		else {
			ROS_ERROR("Problema con IK");
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::shutdown();
	return 0;
}
//Posicion Home 1.0125; 0.007348; 1.4664
void kukaEfectorCallback(const geometry_msgs::Pose & msg) {
	ROS_INFO_STREAM("recibe callback");
	goalPose.orientation.w=msg.orientation.w;
	goalPose.orientation.x=msg.orientation.x;
	goalPose.orientation.y=msg.orientation.y;
	goalPose.orientation.z=msg.orientation.z;
	goalPose.position.x=1.0125+msg.position.x;
	goalPose.position.y=0.007348+msg.position.y;
	goalPose.position.z=1.4664+msg.position.z;
}
