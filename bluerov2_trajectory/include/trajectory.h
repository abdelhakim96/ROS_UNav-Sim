#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Range.h>
#include <iostream>     
#include <fstream>
#include <dynamic_reconfigure/server.h>


using namespace geometry_msgs;
using namespace std;
using namespace ros;

const double deg2rad = M_PI / 180;
const double rad2deg = 180 / M_PI;

std::ifstream inputFile("/traj_extern/wp.txt");

//rqt_reconfig variables

double wp_x, wp_y, wp_z, wp_yaw, absvel, radius;
bool traj_on, pub_on_setpoint_pos,reg_on;
int traj_type;



// Roslaunch param
std::string mocap_topic;

// Publishers
ros::Publisher ref_pos_pub;
ros::Publisher ref_vel_pub;
ros::Publisher ref_yaw_pub;
ros::Publisher setpoint_pos_pub;
ros::Publisher traj_on_pub;
ros::Publisher reg_on_pub;


// Subscribers
ros::Subscriber pos_sub;
ros::Subscriber traj_extern_sub;





geometry_msgs::Vector3 ref_traj_msg;
std::vector<double> current_pos, current_att(3, 0.0);
geometry_msgs::PoseStamped desired_pos;

std_msgs::Float64 ref_yaw_msg;
geometry_msgs::PoseStamped setpoint_pos_msg;
tf::Quaternion current_att_quat, setpoint_att_quat;
tf::Matrix3x3 current_att_mat, R_BI, R_IB;
std_msgs::Bool traj_on_msg, reg_on_msg, inspection_start_msg;
double d_theta, wp_ext_x, wp_ext_y, wp_ext_z, wp_ext_yaw;
bool init_pos;





