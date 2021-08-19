/**
 *
 *
 * Author ROB P1 Group B305
 * AALBORG UNIVERSITY
 * 2014
 *
 * This node controlls everything that is related to the robot controlling.
 * It gets commands from the HQ and sends messages to the turtlebot
 *
 */

/******************************************************************************
 * INCLUDE STATEMENTS FOLLOW
 ******************************************************************************/

#include "ros/ros.h"
#include <vector>

//Twist is the message that the turtlebot needs to know how to drive
#include <geometry_msgs/Twist.h>

//This is the message that the HQ publishes to tell the RC what to tell to the robot
#include <p1Turtle/lin_ang_msg.h>

/******************************************************************************
 * FUNCTION DECLARATIONS FOLLOW
 ******************************************************************************/

/**
 * Function: Publishes a message to the robot telling it at what speed to drive and what speed to turn
 */
void publish(const p1Turtle::lin_ang_msg::ConstPtr&);


/******************************************************************************
 * GLOBAL VARIABLE DECLARATIONS FOLLOW
 ******************************************************************************/

//linear_ means the driving speed of the robot, + values drives frontwards, - values drives backwards
//angulular means how the turtlebot should turn
double linear_, angular_;

//Suscriber to listen to the message from the HQ
ros::Subscriber from_HQ;

//Suscriber to publish the commands to the turtlebot (vel stands for velocity)
ros::Publisher pub_vel;

/******************************************************************************
 * MAIN FUNCTION FOLLOWS
 ******************************************************************************/
 
/**
 * The main function initialises the node, initializes the publisher to send messages to the turtlebot
 * and suscribes to messages from the head quarter
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "RobotController");
	ros::NodeHandle n;

	pub_vel = n.advertise < geometry_msgs::Twist > ("cmd_vel", 10);

	while (ros::ok()) {
		from_HQ = n.subscribe("cmd_from_HQ", 1000, publish);
		ros::spin();
	}
	return 0;
}

/******************************************************************************
 * OTHER FUNCTION DEFINITIONS FOLLOW
 ******************************************************************************/

/**
 * Function: Publishes a message to the robot telling it at what speed to drive and what speed to turn
 */
void publish(const p1Turtle::lin_ang_msg::ConstPtr& msg) {
	ros::Rate loop_rate(100);

	//Creates a new message of type Twist to publish it to the turtlebot
	geometry_msgs::Twist vel;
	vel.linear.x = msg->linear;
	vel.angular.z = msg->angular;

	//Publishing the message to the turtlebot
	pub_vel.publish(vel);
	
	loop_rate.sleep();

	return;

}
