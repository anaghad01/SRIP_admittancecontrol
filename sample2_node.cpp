#include <ros/ros.h>
#include <cmath>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
//#include <math/gzmath.hh>

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
 ros::init(argc, argv, "sample1_node");
 
 ros::NodeHandle n;
 ros::ServiceClient state_sub = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
 //ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
 ros::ServiceClient local_wr_pub = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
 
 gazebo_msgs::GetModelState curr_state;
 //gazebo_msgs::SetModelState objstate;
 gazebo_msgs::ApplyBodyWrench wr;

 ros::Rate rate(100.0);
 ROS_INFO("Initialized");


 ros::Time time_start = ros::Time::now();

 ROS_INFO("started");
 
 wr.request.body_name = "unit_box::link";
 wr.request.reference_frame = "world";
 wr.request.start_time=ros::Time::now();
 curr_state.request.model_name = "unit_box";


 float TIME=0;
 MatrixXd Ak(12,12);
 MatrixXd Bk(12,4);
 MatrixXd Ck(6,12);
 MatrixXd Q= MatrixXd:: Identity(12,12)*10;
 MatrixXd R= MatrixXd:: Identity(6,6)*10;
 MatrixXd K(12,6);
 //MatrixXd L(18,6);

 VectorXd y(6);
 VectorXd u(4);

 VectorXd est_state(12);
 //VectorXd pre_state(18);

 MatrixXd P=MatrixXd:: Identity(12,12);
 MatrixXd I=MatrixXd:: Identity(12,12);

 Ak << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 9.81, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, -9.81, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

 Bk << 0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       1, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 5.999998, 0, 0,
       0, 0, 5.999998, 0,
       0, 0, 0, 5.999998;

 Ck << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;


 /*L << 11.6246, -0.0000, 0.0000, 0.0000, -0.4186, 0.0000,
 -0.0000, 11.6246, -0.0000, 0.4186, 0.0000, -0.0000,
 0.0000 , 0.0000, 11.3682, -0.0000, 0.0000, 0.0000,
 17.6529, -0.0000, 0.0000, 0.0000, -9.0494, 0.0000,
 0.0000, 17.6529, -0.0000 , 9.0494, 0.0000, -0.0000,
 0.0000, -0.0000, 14.6179, -0.0000, 0.0000, -0.0000,
 0.0000, 0.4186, -0.0000, 9.9912, 0.0000, -0.0000,
 -0.4186, 0.0000, 0.0000, 0.0000, 9.9912, 0.0000,
 0.0000, -0.0000, 0.0000, -0.0000, 0.0000, 10.0000,
 0.0000, 0.0000, -0.0000, 0.0001, -0.0000, 0.0000,
 -0.0000, 0.0000 , -0.0000 , -0.0000, 0.0001, -0.0000,
 -0.0000, 0.0000, -0.0000, 0.0000, 0.0000 , 0.0001,
 -0.0000, 0.0000, -0.0000, -0.0497, 0.0000 , -0.0000,
 -0.0000, -0.0000, -0.0000, 0.0000, -0.0497, 0.0000,
 0.0000, -0.0000, 0.0000, -0.0000, 0.0000, -0.0895,
 9.9912, -0.0000 , -0.0000, -0.0000, 0.4186 , -0.0000,
 0.0000, 9.9912, -0.0000, -0.4186, 0.0000, -0.0000,
 0.0000 , -0.0000, 10.0000, -0.0000, 0.0000, -0.0000;*/
 
 float kd_f=8.0, kp_f=6.0;
 float kd_my=8, kp_my=0.9; //while 10 kp=0.01,kd=1.005 //kp=0.45 kd=3
 float kd_y=8, kp_y=0.9;
 float kd_mx=8.0, kp_mx=0.9;
 float kd_x=8, kp_x=0.9;
 double pitch,roll;
 float phi_d, theta_d; //for y-direction

 VectorXd pre_state(12);
 // pre_state << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

 for(int i=100; ros::ok() && i>0; --i){
 // pre_state << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
 P=MatrixXd:: Identity(12,12);
 }

 ROS_INFO("done");
 
 while(ros::ok()){
 state_sub.call(curr_state);
 local_wr_pub.call(wr);
 
 double phi_d=(((-1)/9.8)*(kd_y*(0-curr_state.response.twist.linear.y)+kp_y*(5-curr_state.response.pose.position.y)));
 double theta_d=((1/9.8)*(kd_x*(0-curr_state.response.twist.linear.x)+kp_x*(5-curr_state.response.pose.position.x)));

 double sinr_cosp=2*((curr_state.response.pose.orientation.w * curr_state.response.pose.orientation.x) + (curr_state.response.pose.orientation.y * curr_state.response.pose.orientation.z));
 double cosr_cosp=1-2*((curr_state.response.pose.orientation.x * curr_state.response.pose.orientation.x) + (curr_state.response.pose.orientation.y * curr_state.response.pose.orientation.y));
 double roll=180.0*std::atan2(sinr_cosp, cosr_cosp)/M_PI;
 
 
 float sinp=2.0*(curr_state.response.pose.orientation.w * curr_state.response.pose.orientation.y - curr_state.response.pose.orientation.z * curr_state.response.pose.orientation.x);
 if(std::abs(sinp)>=1) pitch=(180*std::copysign(M_PI/2, sinp))/M_PI;
 else pitch=(180*std::asin(sinp))/M_PI;

 double siny_cosp=2*((curr_state.response.pose.orientation.w * curr_state.response.pose.orientation.z) + (curr_state.response.pose.orientation.x * curr_state.response.pose.orientation.y));
 double cosy_cosp=1-2*((curr_state.response.pose.orientation.y * curr_state.response.pose.orientation.y) + (curr_state.response.pose.orientation.z * curr_state.response.pose.orientation.z));
 float yaw=180.0*std::atan2(siny_cosp, cosy_cosp)/M_PI;
 yaw = 0.0;
 float F_thrust_force = 9.8 + kp_f*(10-curr_state.response.pose.position.z)+kd_f*(0-curr_state.response.twist.linear.z);

 yaw = yaw*M_PI/180;
 roll = roll*M_PI/180;
 pitch = pitch*M_PI/180;

 float FX = F_thrust_force*(std::cos(yaw)*std::sin(pitch) + std::cos(pitch)*std::sin(roll)*std::sin(yaw));
 float FY = F_thrust_force*(std::sin(yaw)*std::sin(pitch) - std::cos(pitch)*std::sin(roll)*std::cos(yaw));
 float FZ = F_thrust_force*(std::cos(roll)*std::cos(pitch));
 

 wr.request.wrench.force.z = FZ;
 wr.request.wrench.force.x = FX;
 wr.request.wrench.force.y = FY;
 //ROS_INFO(" forcez -> %f", FZ);
 //ROS_INFO(" forcex -> %f", FX);
 //ROS_INFO(" forcey -> %f", FY);

 double Control_torque_x ;
 double Control_torque_y ;
 double Control_torque_z ;
 
 yaw = yaw*180/M_PI;
 roll = roll*180/M_PI;
 pitch = pitch*180/M_PI;

 double d=phi_d-roll;
 double s=theta_d-pitch;
 Control_torque_x = kp_mx*(d)+kd_mx*(0.0-curr_state.response.twist.angular.x);
 Control_torque_y = kp_my*(s)+kd_my*(0.0-curr_state.response.twist.angular.y);
 Control_torque_z= kp_my*(0-yaw)+kd_my*(0.0-curr_state.response.twist.angular.z);
 
 wr.request.wrench.torque.y= Control_torque_y;
 wr.request.wrench.torque.x = Control_torque_x;
 
 /*ROS_INFO(" Y pos -> %f", curr_state.response.pose.position.y);
 ROS_INFO(" Y vel -> %f", curr_state.response.pose.position.x);
 ROS_INFO(" Phi D -> %f", phi_d);
 ROS_INFO(" diff -> %f", kp_my*(phi_d-roll));
 ROS_INFO(" Control torque -> %f", Control_torque_x);
 ROS_INFO(" Control torque -> %f", Control_torque_y);
 ROS_INFO(" Angular position (roll) -> %f",roll);
 ROS_INFO(" Angular velocity About X -> %f",curr_state.response.twist.angular.x);*/
 
 // ROS_INFO("************");

 wr.request.duration=ros::Duration(0.01);

 y << curr_state.response.pose.position.x, curr_state.response.pose.position.y, curr_state.response.pose.position.z, roll, pitch, yaw;
 
 u << F_thrust_force, Control_torque_x, Control_torque_y, Control_torque_z;

 //pre_state << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
 //P=MatrixXd:: Identity(18,18);

 est_state= Ak*pre_state+Bk*u;
 P=Ak*P*Ak.transpose()+Q;
 K=P*Ck.transpose()*R.inverse();
 est_state+=K*(y-Ck*est_state);
 P=(I-K*Ck)*P;
 // est_state = (Ak-L*Ck)*pre_state+ L*y + Bk*u;
 //est_state = (Ak-L*Ck)*est_state+ L*y + Bk*u;

 // cout<<(Ak-L*Ck);
 // pre_state=est_state;

 cout<<est_state<<"********************";
 
 }
}

    
