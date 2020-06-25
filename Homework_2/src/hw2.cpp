// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

// include math 
#include <math.h>

using namespace std;

turtlesim::Pose pose;
geometry_msgs::Twist vel_msg;
geometry_msgs::Point goal_point;
//Define a data structure to 3D
struct XYZ{
  float x;
  float y;
  float z;
};
//Declare a variable.Its name is pos_err with XYZ data type
struct XYZ pos_err;

// declare call back function(call back the pose of robot)
void pos_cb(const turtlesim::Pose::ConstPtr& msg)
{
  pose = *msg;
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tutorial_1");
  ros::NodeHandle n;

  // declare publisher & subscriber
  ros::Subscriber pos_sub = n.subscribe<turtlesim::Pose>("turtle1/pose", 10, pos_cb);
  ros::Publisher turtlesim_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
  //input your desired position
  ROS_INFO("Please input (x,y). x>0,y>0");
  cout<<"desired_X:";
  cin>>goal_point.x;
  cout<<"desired_Y:";
  cin>>goal_point.y;
  // setting frequency as 10 Hz
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){

    printf("\ncount : %d\n",count);
    printf("goal x : %f \t y : %f\n",goal_point.x,goal_point.y);
    printf("pose x : %f \t y : %f\n",pose.x,pose.y);

    // Calculate position error(feedback term)
    pos_err.x = goal_point.x - pose.x; //Ex
    pos_err.y = goal_point.y - pose.y; //Ey
    
    /*Your error-driven controller design
    /*vel_msg.linear.x = (Your control input design);
     *vel_msg.angular.z = (Your control input design);*/
    //my design//
    //for angular//
    float dest = atan2f(pos_err.y , pos_err.x); //angle that we need 
    float Et= dest-pose.theta; //the error between the angle we need and the current angle theta

    //for linear//
    float lin= sqrt(pow(pos_err.x,2) + pow(pos_err.y,2)); //distance formula
    vel_msg.linear.x = lin *cos(Et)*1.5; // P controller sets to 1.5
    vel_msg.angular.z = 2*(dest-pose.theta); //P controller sets to 2 (increase the speed)
    
    //vel_msg.linear.x= hypotf(pos_err.x,pos_err.y)*cos(Et);

    turtlesim_pub.publish(vel_msg);

    count ++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}



