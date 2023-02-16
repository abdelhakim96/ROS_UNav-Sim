/**
 * @file   trajectory.cpp
 * @author Hakim Amer / Mohit Mehindratta
 * @date   
 *  Trajectory generation node, contains 4 modes:
 1- position hold
 2- Go to setposition 
 3- Circular trajectory 
 4- Custom trajectory read from a text file 
 * @copyright
 * Copyright (C) 2022.
 */



#include <trajectory.h>   // Adding header file: contains all variable types definitions and subscribers/publishers 
#include <bluerov2_trajectory/set_trajectoryConfig.h>
double sampleTime = 0.5; 
// Callback functions
//dynamicReconfigureCallback Retreives all the data from rqt_reconfigure GUI 

void dynamicReconfigureCallback(bluerov2_trajectory::set_trajectoryConfig& config, uint32_t level)  

{
    traj_on = config.traj_on;                              // Flag for activation of trajectory script
    pub_on_setpoint_pos = config.pub_on_setpoint_position; // Flag for publishing trajectory on mavros topic
    traj_type = config.traj_type;                          // Flag for publishing trajectory on mavros topic
    wp_x=config.wp_x;                                      // User specified waypoint x-coordinate
    wp_y=config.wp_y;                                      // User spcified waypoint y-coordinate
    wp_z=config.wp_z;                                      // User specified waypoint z-coordinate
    wp_yaw=config.wp_yaw;                                  // User specified yaw angle 
    radius = config.des_radius;                            // User specified radius of turn for circle trajectory
    absvel = config.des_velocity;                          // Velocity of ROV during circle trajectory
    reg_on = config.reg_on;

                         
}




// Callback for current ROV position 
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    current_att_quat = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
    current_att_mat.setRotation(current_att_quat);
    current_att_mat.getRPY(current_att[0], current_att[1], current_att[2]);


}

// Callback for desired ROV position 
void traj_extern_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    desired_pos = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rov_trajectory");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<bluerov2_trajectory::set_trajectoryConfig> server;
    dynamic_reconfigure::Server<bluerov2_trajectory::set_trajectoryConfig>::CallbackType f;

    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    // Roslaunch parameters
    

    // Publishers
    ref_pos_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/position", 1);   // reference position for ROV in world frame [m]
    ref_vel_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/velocity", 1);    //reference velocity for ROV in ROV body frame  [m/s]
    ref_yaw_pub = nh.advertise<std_msgs::Float64>("ref_trajectory/yaw", 1);              // reference yaw angle for ROV [deg]
    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1); //reference position for ROV in world frame (for MAVROS) [m]
    traj_on_pub = nh.advertise<std_msgs::Bool>("trajectory_on", 1);   // Boolean indicating if Traj_on button is clicked (for recording data)
    reg_on_pub = nh.advertise<std_msgs::Bool>("regression_on", 1);   // Boolean indicating if GP regression is activated
    
 
    // Subscribers
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/mocap/position", 1, pos_cb);  // get current ROV position (ground truth)
    traj_extern_sub = nh.subscribe<geometry_msgs::PoseStamped>("/trajectory_extern", 1, traj_extern_cb); // get desired trajectory from external source
  


     // Sampling time for ros node
    ros::Rate rate(1 / sampleTime);


    while (ros::ok())
    {


        traj_on_msg.data = traj_on;
        ROS_INFO(" traj_on %f",  traj_on);


        if (traj_on)
        {
           
  
        if (traj_type == 0)  //  Keep current position
           {
             
             ROS_INFO("--------Position Hold!--------");
             
             if (init_pos)
             {
             ref_traj_msg.x = current_pos.at(0); 
             ref_traj_msg.y = current_pos.at(1);             
             ref_traj_msg.z = current_pos.at(2);
             ref_yaw_msg.data = current_att[2];
             init_pos=0;
             }

           }
           
        if (traj_type == 1)  // Set point navigation defined by rqt_reconfigure (user)
            {
               ROS_INFO("--------Waypoint navigation: Go to Setpoint!--------");
               ref_traj_msg.x  = wp_x;
               ref_traj_msg.y  = wp_y;             
               ref_traj_msg.z  = wp_z;
               ref_yaw_msg.data = wp_yaw* ((M_PI)/180.0);
               //init_pos=1;
            }

        if (traj_type == 2)  // Circlular trajectory with specified radius,centre and velocity
            {
               ROS_INFO("--------Circle selected!--------");
               d_theta = absvel * sampleTime * radius + d_theta;
               ref_traj_msg.x = wp_x + absvel * cos (d_theta);
               ref_traj_msg.y = wp_y + absvel * sin (d_theta);
               ref_traj_msg.z = wp_z;
               ref_yaw_msg.data = wp_yaw;
              // bool init_pos=1;
            }
        if (traj_type == 3)  // Read reference trajectory from a textfile
            {
               ROS_INFO("--------External trajectory: Reading trajectory from txt file!--------"); 
               while (inputFile >> wp_ext_x >> wp_ext_y >> wp_ext_z >> wp_ext_yaw)
                      {
                        ref_traj_msg.x =  wp_ext_x ;                                                                                                                                                                                                                                                                                                                              
                        ref_traj_msg.y =  wp_ext_y ;
                        ref_traj_msg.z =  wp_ext_z ; 
                        ref_yaw_msg.data = wp_ext_yaw;
                        ref_pos_pub.publish(ref_traj_msg);   
                       }  
             //  bool init_pos=1;        
            }
        
        }
        reg_on_msg.data = reg_on;
        reg_on_pub.publish(reg_on_msg);
        ref_pos_pub.publish(ref_traj_msg);
        ref_yaw_pub.publish(ref_yaw_msg);
        traj_on_pub.publish(traj_on_msg);
        ros::spinOnce();
        rate.sleep();
   
    }

}
