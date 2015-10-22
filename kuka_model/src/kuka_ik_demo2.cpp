//Basado en el ejemplo de kinematic_model_tutorial para el robot pr2.

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>


geometry_msgs::Pose goalPose;
bool updatePose=false;
void kukaIkCallback(const geometry_msgs::PoseStamped & msg);

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

	// Publisher for KUKA model joint position from IK solver
	ros::Publisher kukaJointPub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

	// Suscriber for goal position (InteractiveMarker)
	ros::Subscriber kukaIkGoalSub = n.subscribe("goal_pose", 50, kukaIkCallback);


	// JointState Msg for KUKA model from IK solver
	sensor_msgs::JointState jointMsg;
	jointMsg.name.resize(6);
	jointMsg.position.resize(6);
	jointMsg.velocity.resize(6);
	for(std::size_t i=0; i < joint_names.size(); ++i){
		jointMsg.name[i] = joint_names[i].c_str();
		jointMsg.position[i] = joint_values[i];
	}
	// Publish init joint state
	ros::Duration(1).sleep();
	jointMsg.header.stamp = ros::Time::now();
	kukaJointPub.publish(jointMsg);

	// Mensaje
	while (ros::ok()) {

		if (updatePose){
			// IK
			kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

			bool found_ik = kinematic_state->setFromIK(joint_model_group, goalPose, 5, 0.01);

			// IK solution (if found):
			if (found_ik) {
				// Actualizar joint_values
				kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

				// for(std::size_t i=0; i < joint_names.size(); ++i){
				// 	jointMsg.position[i] = joint_values[i];
				// 	//ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
				// }
				jointMsg.position[0] = joint_values[0];
				jointMsg.position[1] = joint_values[1];
				jointMsg.position[2] = joint_values[2];
				//jointMsg.position[3] = 0.0; //GIRO KINECT
				jointMsg.position[3] = joint_values[3]; //GIRO KINECT
				jointMsg.position[4] = joint_values[4];
				jointMsg.position[5] = joint_values[5];
			}
			else {
				ROS_ERROR_THROTTLE(2,"INVERSE KINEMATIC IS NOT FEASIBLE!");
			}
			updatePose=false;
		}
		// Publicar mensaje para KUKA model
		jointMsg.header.stamp = ros::Time::now();

		kukaJointPub.publish(jointMsg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::shutdown();
	return 0;
}
//Posicion Home 1.0125; 0.007348; 1.4664
void kukaIkCallback(const geometry_msgs::PoseStamped & msg) {
	ROS_INFO("IK goal pose updated");
	//// ORIENTATION MODO EFECTOR SIEMPRE PERPENDICULAR A PLANO
	// goalPose.orientation.x =1.0;
	// goalPose.orientation.y =0.0;
	// goalPose.orientation.z =0.0;
	// goalPose.orientation.w =1.0;
	//// ORIENTATION MODO EFECTOR LIBRE
	goalPose.orientation=msg.pose.orientation;
	//// SET POSITION EFECTOR
	goalPose.position=msg.pose.position;
	updatePose=true;
}
