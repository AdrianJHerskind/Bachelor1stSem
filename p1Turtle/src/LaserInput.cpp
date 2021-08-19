/**
 *
 *
 * Author ROB P1 Group B305
 * AALBORG UNIVERSITY
 * 2014
 *
 * This node takes the input from a laser scan, evaluates the data and sends a message
 * to the HQ node
 *
 */

/******************************************************************************
 * INCLUDE STATEMENTS FOLLOW
 ******************************************************************************/

#include "ros/ros.h"
#include <vector>

// Standard message to deliver data from a laser scan.
#include "sensor_msgs/LaserScan.h"

/**
 * This message works with bools represented by integers to describe the different states
 * of obstacles in certain directions. 0 means false --> no obstacle, 1 means true --> obstacle
 * Fields:
 * front_obstacle
 * left_obstacle
 * right_obstacle
 */
#include "p1Turtle/obst_msg.h"

/******************************************************************************
 * FUNCTION DECLARATIONS FOLLOW
 ******************************************************************************/

/**
 * Function: Loops and calls other functions, which read in data and evaluate it
 */
void processLaser(ros::NodeHandle n);

/**
 * Function: Reads the data from the laser scanner and puts the ranges in a vector.
 */
void fillInputVector(const sensor_msgs::LaserScan::ConstPtr&);

/**
 * Function: Checks the ranges in the middle two quartes infront of the laser and checks if an obstacle is there
 * If this is the case it sets the value of front_obstacle to 1
 */
void checkFront();

/**
 * Function: Checks the ranges in the left quarter of the laser and checks if an obstacle is there
 * If this is the case it sets the value of left_obstacle to 1
 */
void checkLeft();

/**
 * Function: Checks the ranges in the right quarter of the laser and checks if an obstacle is there
 * If this is the case it sets the value of right_obstacle to 1
 */
void checkRight();

/**
 * Function: Prints the smallest and largest number in the inputVector with the ranges
 * This function is mainly a tool for the programmer to surveill the data
 */
void printMinAndMax();

/******************************************************************************
 * GLOBAL VARIABLE DECLARATIONS FOLLOW
 ******************************************************************************/

//The vector which will store the ranges received from the laser scan
std::vector<float> inputVector;

//Suscriber which will be used to suscribe to the laser scan
ros::Subscriber sub;

//Publisher which is used throughout the node to send a message to the HQ about possible obstacles
ros::Publisher pub_msg;

//An obstacle message which is used throughout the node to be published
p1Turtle::obst_msg my_obst_msg;

//The following three are values which give tolerance values about how far the obstacle may be away on each side of the robot
float frontTolerance;
float rightTolerance;
float leftTolerance;

/******************************************************************************
 * MAIN FUNCTION FOLLOWS
 ******************************************************************************/

/**
 * 1) The main function initialises the node
 * 2) The main function creates a node handler
 * 3) The main function defines to advertise a message of type obst_msg
 * 4) The main function calls the function processLaser which will do all the work
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "LaserInput");
	ros::NodeHandle n;

	leftTolerance = 0.5;
	rightTolerance = 0.5;
	frontTolerance = 0.5;

	pub_msg = n.advertise < p1Turtle::obst_msg > ("obstacle_msg", 10);
	processLaser(n);

	return 0;
}

/******************************************************************************
 * OTHER FUNCTION DEFINITIONS FOLLOW
 ******************************************************************************/

/**
 * Function: Loops and calls other functions, which read in data and evaluate it
 */
void processLaser(ros::NodeHandle n)
{
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		sub = n.subscribe("scan", 10, fillInputVector);

		//Calls functions to check for obstacles in front, left and right.
		checkFront();
		checkLeft();
		checkRight();

		printMinAndMax();

		loop_rate.sleep();

		//Publish the message containing information about possible obstacles
		pub_msg.publish(my_obst_msg);

		ros::spinOnce();
	}
}

/**
 * Function: Reads the data from the laser scanner and puts the ranges in a vector.
 *
 */
void fillInputVector(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//Deletes the vector from last scan
	inputVector.clear();

	//Goes through the vector and fills it with values, it should always be 512 values for this specific sensor
	for (int i = 0; i < msg->ranges.size(); i++)
	{
		float inputTemp = msg->ranges[i];

		if (inputTemp < msg->range_min) //if the range value is smaller than it is possible it is set to 0
		{
			inputTemp = 0.0;
		}
		else if (inputTemp > msg->range_max) //if the range value is larger than it is possible it is set to 0
		{
			inputTemp = 0.0;
		}

		//the value gets put into the vector
		inputVector.push_back(inputTemp);
	}

	//For Information for the programmer only: it says how many values it puts in the vector
	ROS_INFO("I got a vector with the size of: ");
	ROS_INFO("[%d]", inputVector.size());
}

/**
 * Function: Checks the ranges in the middle two quartes infront of the laser and checks if an obstacle is there
 * If this is the case it sets the value of front_obstacle to 1
 */
void checkFront()
{
	//512 values spread over 180 degrees
	//Every value has an angle increment of 0.3515625 degrees
	//After splitting the data into quartes, only the value of the middle two quarters is checked
	//The values start from index 128 and go til value 383
	//But for this case no specific numbers are used, because then the sensor can easily be exchanged.
	my_obst_msg.front_obstacle = 0;

	for (int i = inputVector.size() / 4; i < (inputVector.size() / 4) * 3; i++)
	{

		if (inputVector.empty())
		{
			ROS_WARN("EMPTY");
			break;
		}
		if (inputVector[i] >= 0.1 && inputVector[i] < frontTolerance)
		{
			my_obst_msg.front_obstacle = 1;
			break;
		}
	}
}

/**
 * Function: Checks the ranges in the left quarter of the laser and checks if an obstacle is there
 * If this is the case it sets the value of left_obstacle to 1
 */
void checkLeft()
{
	//512 values spread over 180 degrees
	//Every value has an angle increment of 0.3515625 degrees
	//After splitting the data into quartes, only the value of the left quarter is checked
	//Because the scan is counter clockwise -->
	//The values start from index 384 and go til value 511
	//But for this case no specific numbers are used, because then the sensor can easily be exchanged.

	my_obst_msg.left_obstacle = 0;

	for (int i = (inputVector.size() / 4) * 3; i < inputVector.size(); i++)
	{
		if (inputVector.empty())
		{
			ROS_WARN("EMPTY");
			break;
		}
		if (inputVector[i] >= 0.1 && inputVector[i] < leftTolerance)
		{
			my_obst_msg.left_obstacle = 1;
			break;
		}
	}
}

/**
 * Function: Checks the ranges in the right quarter of the laser and checks if an obstacle is there
 * If this is the case it sets the value of right_obstacle to 1
 */
void checkRight()
{
	//512 values spread over 180 degrees
	//Every value has an angle increment of 0.3515625 degrees
	//After splitting the data into quartes, only the value of the right quarter is checked
	//Because the scan is counter clockwise -->
	//The values start from index 0 and go til value 127
	//But for this case no specific numbers are used, because then the sensor can easily be exchanged.

	my_obst_msg.right_obstacle = 0;

	for (int i = 0; i < inputVector.size() / 4; i++)
	{
		if (inputVector.empty())
		{
			ROS_WARN("EMPTY");
			break;
		}
		if (inputVector[i] < rightTolerance && inputVector[i] >= 0.1)
		{
			my_obst_msg.right_obstacle = 1;
			break;
		}
	}

}

/**
 * Function: Prints the smallest and largest number in the inputVector with the ranges
 * This function is mainly a tool for the programmer to surveill the data
 */
void printMinAndMax()
{
	float minimum = 5.2;
	float maximum = 0.0;
	for (int j = 0; j < inputVector.size() - 1; j++)
	{
		if (inputVector.empty())
		{
			ROS_WARN("EMPTY");
			break;
		}
		if (inputVector[j] < minimum && inputVector[j] > 0.1)
		{
			minimum = inputVector[j];
		}

		if (inputVector[j] > maximum)
		{
			maximum = inputVector[j];
		}
	}

	ROS_INFO("Minimum: [%f]", minimum);
	ROS_INFO("Maximum: [%f]", maximum);
}
