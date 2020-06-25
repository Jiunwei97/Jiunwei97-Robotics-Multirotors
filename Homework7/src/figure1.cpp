#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>


// Callback of timer
// Broadcaster transform from A to B and B to C
void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;


  //rotate C_B with 60 degrees
  //C_B Eigen::Quaterniond w x y z 
  //q=[cos(A/2),n_x*sin(A/2),n_y*sin(A/2),n_z*sin(A/2)] (A=60 degree, rotate about z-axis)
  Eigen::Quaterniond q_C_B(std::sqrt(3)/2, 0, 0, 0.5);
  Eigen::Vector3d v_C_B(1, 0, 0);

  //tf::Quaternion x y z w
  tf::Transform tf_C_B;
  tf::Quaternion tf_q_C_B(q_C_B.x(), q_C_B.y(), q_C_B.z(), q_C_B.w());
  tf::Vector3 tf_v_C_B(v_C_B(0), v_C_B(1), v_C_B(2));

  tf_C_B.setOrigin(tf_v_C_B);
  tf_C_B.setRotation(tf_q_C_B);

  br.sendTransform(tf::StampedTransform(tf_C_B,
                                        ros::Time::now(),
                                        "C", // paranet frame ID
                                        "B"));

  //rotate B_A with 90 degrees
  //q=[cos(A/2),n_x*sin(A/2),n_y*sin(A/2),n_z*sin(A/2)] (A=90 degree, rotate about z-axis)
  Eigen::Quaterniond q_B_A(std::sqrt(2)/2,0,0,std::sqrt(2)/2);
  Eigen::Vector3d v_B_A(1,0,0);

  tf::Transform tf_B_A;
  tf::Quaternion tf_q_B_A(q_B_A.x(), q_B_A.y(), q_B_A.z(), q_B_A.w());
  tf::Vector3 tf_v_B_A(v_B_A(0), v_B_A(1), v_B_A(2));

  tf_B_A.setOrigin(tf_v_B_A);
  tf_B_A.setRotation(tf_q_B_A);

  br.sendTransform(tf::StampedTransform(tf_B_A,
                                        ros::Time::now(),
                                        "B", // paranet frame ID
                                        "A"));

  //The question asks for the output vector AC (C_A1)
  //q=[cos(A/2),n_x*sin(A/2),n_y*sin(A/2),n_z*sin(A/2)]
  //for this case, it's q=(cos(15)),0,0,sin(15) unit in degree, use wolfalpha to get the exact result and move the term.
  Eigen::Quaterniond q_C_A1((std::sqrt(6)-std::sqrt(2))/4,0,0,(std::sqrt(6)+std::sqrt(2))/4);
  //use the geometry of the triangle to find x and y ie: x=1+1cos60=1.5 , y=sin60
  Eigen::Vector3d v_C_A1(1.5,std::sqrt(3)/2,0);

  tf::Transform tf_C_A1;
  tf::Quaternion tf_q_C_A1(q_C_A1.x(), q_C_A1.y(), q_C_A1.z(), q_C_A1.w());
  tf::Vector3 tf_v_C_A1(v_C_A1(0), v_C_A1(1), v_C_A1(2));

  tf_C_A1.setOrigin(tf_v_C_A1);
  tf_C_A1.setRotation(tf_q_C_A1);

  br.sendTransform(tf::StampedTransform(tf_C_A1,
                                        ros::Time::now(),
                                        "C",
                                        "A1"));

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ncrl_tf_learning");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}