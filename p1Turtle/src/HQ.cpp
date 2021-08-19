/**
 *
 *
 * Author ROB P1 Group B305 
 * AALBORG UNIVERSITY
 * 2014
 *
 * This node acts as a medium point between laser input and robot output
 * The HQ will receive informations from the laser input which tell about possible obstacles in its path
 * The HQ will also send messages to the RobotController to steer the robot,
 * reacting accordingly to the input from the laser
 *
 */

/******************************************************************************
 * INCLUDE STATEMENTS FOLLOW
 ******************************************************************************/

#include "ros/ros.h"
#include <string>
#include <iostream>

#include <geometry_msgs/Twist.h>

/**
 * This message works with bools represented by integers to describe the different states
 * of obstacles in certain directions. 0 means false --> no obstacle, 1 means true --> obstacle
 * Fields:
 * front_obstacle
 * left_obstacle
 * right_obstacle
 */
#include <p1Turtle/obst_msg.h>

/**
 * This message sends two float values representing linear and angular values
 */
#include <p1Turtle/lin_ang_msg.h>

/******************************************************************************
 * FUNCTION DECLARATIONS FOLLOW
 ******************************************************************************/

/**
 * Function: Receives a message from the HQ node and sets the values of the message into the global variables of the node
 */
void setInput(const p1Turtle::obst_msg::ConstPtr&);

/**
 * Function: Goes into a loop which receives messages from the LaserInput and sends messages to the RobotController
 */
void drive(ros::NodeHandle);

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops
 */
void scenario1();

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops.
 * If the obstacle disappears it will drive again.
 */
void scenario2();

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops.
 * The robot turns to the right until the obstacle is out of the way, then it drives again
 */
void scenario3();

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops.
 * The robot waits a certain amount of time, if during that time the obstacle disappears the robot will drive again.
 * If not then the robot will turn to the right until it can drive straight again.
 */
void scenario4();

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops.
 * The robot waits a certain amount of time, if during that time the obstacle disappears the robot will drive again.
 * If not then the robot will check on the left and on the right for obstacles and will decide in which direction to turn
 */
void scenario5();

/**
 * Function: Sets the speed and turn rate to 0
 */
void stopEmergency();

/**
 * Function: Sets the speed to a value and the turn rate to 0
 */
void start();

/**
 * Function: Sets the turning value to a positive value
 */
void turnLeft();

/**
 * Function: Sets the turning value to a negative value
 */
void turnRight();

/******************************************************************************
 * GLOBAL VARIABLE DECLARATIONS FOLLOW
 ******************************************************************************/

//Speed at which to drive, + values drives forwards, - values drives backwards
double _linear = 0.0;
//Speed at which to turn
double _angular = 0.0;

//Bools represented by integers, 0 means false -> no obstacle, 1 means true -> obstacle
int front_obstacle = 0;
int left_obstacle = 0;
int right_obstacle = 0;

//
int wait = 0;

ros::Publisher to_RC;
ros::Subscriber from_LI;

/******************************************************************************
 * MAIN FUNCTION FOLLOWS
 ******************************************************************************/

/**
 * The main function advertises a message and calls the drive function
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "HQ");
	ros::NodeHandle n;

	to_RC = n.advertise < p1Turtle::lin_ang_msg > ("cmd_from_HQ", 10);
	drive(n);

	return 0;
}

/******************************************************************************
 * OTHER FUNCTION DEFINITIONS FOLLOW
 ******************************************************************************/

/**
 * Function: Receives a message from the HQ node and sets the values of the message into the global variables of the node
 */
void setInput(const p1Turtle::obst_msg::ConstPtr& msg)
{
	front_obstacle = msg->front_obstacle;
	left_obstacle = msg->left_obstacle;
	right_obstacle = msg->right_obstacle;

}

/**
 * Function: Goes into a loop which receives messages from the LaserInput and sends messages to the RobotController
 */
void drive(ros::NodeHandle n)
{
	char selection;

	do
	{
		std::cout << "Please select a mode for the turtlebot to run in \n";
		std::cout << "1) Drive and stop when obstacle \n";
		std::cout << "2) Drive -> Stop and Go when obstacle \n";
		std::cout << "3) Drive and stop for obstacle, then turn right \n";
		std::cout
				<< "4) Drive and stop for obstacle, wait, either drive or turn right \n";
		std::cout
				<< "5) Drive and stop for obstacle, wait, either drive or check l/r, then turn \n";
		std::cin >> selection;

	} while (selection != '1' && selection != '2' && selection != '3'
			&& selection != '4' && selection != '5');

	while (ros::ok())
	{
		from_LI = n.subscribe("obstacle_msg", 1000, setInput);

		ros::Rate loop_rate(10);

		switch (selection)
		{
		case '1':
			scenario1();
		break;

		case '2':
			scenario2();
		break;
		case '3':
			scenario3();
		break;
		case '4':
			scenario4();
		break;
		case '5':
			scenario5();
		break;
		}

		if (wait == -1)
		{
			break;
		}

		p1Turtle::lin_ang_msg vel;
		vel.linear = _linear;
		vel.angular = _angular;
		to_RC.publish(vel);

		loop_rate.sleep();
		ros::spinOnce();

	}
}

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops
 */
void scenario1()
{
	if (front_obstacle == 1)
	{
		stopEmergency();
		wait = -1;
	}
	else
	{
		start();
	}
}

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops.
 * If the obstacle disappears it will drive again.
 */
void scenario2()
{
	if (front_obstacle == 1)
	{
		stopEmergency();
	}
	else
	{
		start();
	}
}

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops.
 * The robot turns to the right until the obstacle is out of the way, then it drives again
 */
void scenario3()
{
	if (front_obstacle == 1)
	{
		stopEmergency();
		turnRight();
	}
	else
	{
		start();
	}
}

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops.
 * The robot waits a certain amount of time, if during that time the obstacle disappears the robot will drive again.
 * If not then the robot will turn to the right until it can drive straight again.
 */
void scenario4()
{
	if (front_obstacle == 0)
	{
		if (wait >= 30 && wait < 42)
		{
			wait++;
		}
		else
		{
			wait = 0;
		}
		ROS_WARN("NO OBSTACLE");
		start();
	}
	else
	{
		if (front_obstacle == 1 && wait < 30)
		{
			stopEmergency();
			wait++;

		}
		else if (front_obstacle == 1 && wait >= 30)
		{
			wait = 30;
			turnRight();
		}
	}
}

/**
 * Function: A scenario for the turtlebot, containing certain elements of reaction and interaction
 * 
 * What happens: 
 * The robot drives forward until it comes across an obstacle, then it stops.
 * The robot waits a certain amount of time, if during that time the obstacle disappears the robot will drive again.
 * If not then the robot will check on the left and on the right for obstacles and will decide in which direction to turn
 */
void scenario5()
{
	if (front_obstacle == 0)
	{
		if (wait < 42 && wait >= 30)
		{
			wait++;
		}
		else
		{
			wait = 0;
		}
		ROS_WARN("NO OBSTACLE");
		start();
	}
	if (front_obstacle == 1 && wait < 30)
	{
		stopEmergency();
		wait++;

	}
	if (front_obstacle == 1 && wait >= 30)
	{
		wait = 30;
		ROS_ERROR("STOP OBSTACLE IN FRONT");
		stopEmergency();
		if (left_obstacle == 1 && right_obstacle == 1)
		{
			turnRight();
			ROS_WARN("left and right");
		}
		if (left_obstacle == 0 && right_obstacle == 0)
		{
			turnRight();
			ROS_WARN("NON");
		}
		if (left_obstacle == 0 && right_obstacle == 1)
		{
			ROS_WARN("TURNING LEFT");
			turnLeft();
		}
		if (left_obstacle == 1 && right_obstacle == 0)
		{
			ROS_WARN("TURNING RIGHT");
			turnRight();
		}

	}
}

/**
 * Function: Sets the speed and turn rate to 0
 */
void stopEmergency()
{
	_angular = 0.0;
	_linear = 0.0;
}

/**
 * Function: Sets the speed to a value and the turn rate to 0
 */
void start()
{
	_linear = 0.2;
	_angular = 0.0;
}

/**
 * Function: Sets the turning value to a positive value and the speed to 0
 * In later implementations the robot can also drive while its turning
 */
void turnLeft()
{
	_linear = 0.0;
	_angular = 0.5;
}

/**
 * Function: Sets the turning value to a negative value and the speed to 0
 * In later implementations the robot can also drive while its turning
 */
void turnRight()
{
	_linear = 0.0;
	_angular = -0.5;
}
