/**
 * @file   nmpc_bluerov2_2-D_main.cpp
 * @author Mohit Mehindratta / Hakim Amer
 * @date   July 2022
 *
 * @copyright
 * Copyright (C) 2022.
 */

#include <nmpc_bluerov2_2D_main.h>

#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
using namespace ros;
double sampleTime = 0.02;

mavros_msgs::State current_state_msg;




Eigen::MatrixXf pinv(const Eigen::MatrixXf &input, double eps = std::numeric_limits<double>::epsilon())
{

   


    JacobiSVD<Matrix<float, 2, 3>> svd(input, ComputeFullV | ComputeFullU);
    auto s = svd.singularValues();

    cout<< s.rows() << s.cols()  << endl;

    
    for (int i = 0; i < std::min(2, 3); i++)
    {

        cout << "Begin for" << endl;
        if (s(i) < eps )
        {
            s(i) = 0;
        }
        else
        {
            s(i) = 1 / s(i);
        }

        cout<< s(i) <<endl;

    }

    

    return svd.matrixV() * s.asDiagonal() * svd.matrixU().transpose();
}


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_msg = *msg;
}
void ref_position_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void ref_velocity_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_velocity << msg->x, msg->y, msg->z;
}
void ref_yaw_cb(const std_msgs::Float64::ConstPtr& msg)
{
    ref_yaw_rad = msg->data;
}

void pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    current_vel_rate = {msg->twist.twist.linear.x,
                        msg->twist.twist.linear.y,
                        msg->twist.twist.linear.z,
                        msg->twist.twist.angular.x,
                        msg->twist.twist.angular.y,
                        msg->twist.twist.angular.z};


    current_att_mat.setRotation(current_att_quat);
    //current_att_mat.getRPY(roll, pitch, yaw);
    current_att_mat.getRPY(pitch, roll, yaw);
    current_pos_att = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw};
}



void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_body = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}


void dist_Fx_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fx.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fx.predInit && dist_Fx.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fx estimates! \n";
        dist_Fx.print_predInit = 0;
    }
}
void dist_Fy_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fy.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fy.predInit && dist_Fy.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fy estimates! \n";
        dist_Fy.print_predInit = 0;
    }
}
void dist_Fz_predInit_cb(const std_msgs::Bool::ConstPtr& msg)
{
    dist_Fz.predInit = msg->data;
    if (nmpc_struct.verbose && dist_Fz.predInit && dist_Fz.print_predInit == 1)
    {
        std::cout << "Prediction initialized for Fz estimates! \n";
        dist_Fz.print_predInit = 0;
    }
}

void dist_Fx_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fx.predInit)
    {
        dist_Fx.data.clear();
        dist_Fx.data.insert(dist_Fx.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fx.data = dist_Fx.data_zeros;
}
void dist_Fy_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fy.predInit)
    {
        dist_Fy.data.clear();
        dist_Fy.data.insert(dist_Fy.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fy.data = dist_Fy.data_zeros;
}
void dist_Fz_data_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (use_dist_estimates && dist_Fz.predInit)
    {
        dist_Fz.data.clear();
        dist_Fz.data.insert(dist_Fz.data.end(), msg->data.begin(), msg->data.end());
    }
    else
        dist_Fz.data = dist_Fz.data_zeros;
}

void NMPC_PC::publish_rpyFz(struct command_struct& commandstruct)
{
/*
    geometry_msgs::Wrench nmpc_wrench_msg;


    
    nmpc_wrench_msg.force.x =    commandstruct.control_wrench_vec[0];
    nmpc_wrench_msg.force.y =    commandstruct.control_wrench_vec[1];
    nmpc_wrench_msg.force.z =    commandstruct.control_wrench_vec[2];
    nmpc_wrench_msg.torque.z =   -commandstruct.control_wrench_vec[3];

    nmpc_ctrl_pub.publish(nmpc_wrench_msg);

*/

    std_msgs::Float64MultiArray wrench_msg;
    wrench_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    wrench_msg.layout.dim[0].size = commandstruct.control_wrench_vec.size();
    wrench_msg.layout.dim[0].stride = 1;
    wrench_msg.layout.dim[0].label = "Fx, Fy, Fz, Mz";
    wrench_msg.data.clear();
    wrench_msg.data.insert(
        wrench_msg.data.end(), commandstruct.control_wrench_vec.begin(), commandstruct.control_wrench_vec.end());
    nmpc_cmd_wrench_pub.publish(wrench_msg);


    

    std_msgs::Float64 exe_time_msg;
    exe_time_msg.data = commandstruct.exe_time;
    nmpc_cmd_exeTime_pub.publish(exe_time_msg);

    std_msgs::Float64 kkt_tol_msg;
    kkt_tol_msg.data = commandstruct.kkt_tol;
    nmpc_cmd_kkt_pub.publish(kkt_tol_msg);

    std_msgs::Float64 obj_val_msg;
    obj_val_msg.data = commandstruct.obj_val;
    nmpc_cmd_obj_pub.publish(obj_val_msg);

}

int main(int argc, char** argv)
{
    //MatrixXf m(2,3);
     //m << 3, 2 ,2,
     //     2, 3,-2;

    //MatrixXf a(2,2);
     //a << -3, 4,
      //    2, 5;

    //Matrix<double, 2, 3> randM {{3.0, 2.0, 2.0}, {2.0, 3.0,-2.0},};  

    //std::cout<< pinv(m) << std::endl;
    
    // std::cin.get();

    ros::init(argc, argv, "poseaware_nmpc_pc_learning");
    ros::NodeHandle nh;

    ros::param::get("mocap_topic_part", mocap_topic_part);
    ros::param::get("dist_Fx_predInit_topic", dist_Fx_predInit_topic);
    ros::param::get("dist_Fy_predInit_topic", dist_Fy_predInit_topic);
    ros::param::get("dist_Fz_predInit_topic", dist_Fz_predInit_topic);
    ros::param::get("dist_Fx_data_topic", dist_Fx_data_topic);
    ros::param::get("dist_Fy_data_topic", dist_Fy_data_topic);
    ros::param::get("dist_Fz_data_topic", dist_Fz_data_topic);



    // ----------
    // Subscribers
    // ----------

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ref_position_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/position", 1, ref_position_cb);
    ref_velocity_sub = nh.subscribe<geometry_msgs::Vector3>("ref_trajectory/velocity", 1, ref_velocity_cb);
    ref_yaw_sub = nh.subscribe<std_msgs::Float64>("ref_trajectory/yaw", 1, ref_yaw_cb);
    pos_sub = nh.subscribe<nav_msgs::Odometry>("/airsim_node/RovSimple/odom_local_ned", 1, pos_cb);
    //vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/mocap/velocity_body", 1, vel_cb);
    dist_Fx_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fx_predInit_topic, 1, dist_Fx_predInit_cb);
    dist_Fy_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fy_predInit_topic, 1, dist_Fy_predInit_cb);
    dist_Fz_predInit_sub = nh.subscribe<std_msgs::Bool>(dist_Fz_predInit_topic, 1, dist_Fz_predInit_cb);
    dist_Fx_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fx_data_topic, 1, dist_Fx_data_cb);
    dist_Fy_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fy_data_topic, 1, dist_Fy_data_cb);
    dist_Fz_data_sub = nh.subscribe<std_msgs::Float64MultiArray>(dist_Fz_data_topic, 1, dist_Fz_data_cb);

    // ----------
    // Publishers
    // ----------
    nmpc_cmd_wrench_pub = nh.advertise<std_msgs::Float64MultiArray>("/bluerov2/wrench", 1, true);
    nmpc_cmd_exeTime_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/exeTime", 1, true);
    nmpc_cmd_kkt_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/kkt", 1, true);
    nmpc_cmd_obj_pub = nh.advertise<std_msgs::Float64>("outer_nmpc_cmd/obj", 1, true);

    s_sdot_pub = nh.advertise<std_msgs::Float64MultiArray>("outer_nmpc_cmd/s_sdot", 1, true);

    nmpc_struct.U_ref.resize(NMPC_NU);
    nmpc_struct.W.resize(NMPC_NY);

    // Roslaunch parameters
    ros::param::get("verbose", nmpc_struct.verbose);
    ros::param::get("yaw_control", nmpc_struct.yaw_control);
    ros::param::get("online_ref_yaw", online_ref_yaw);
    ros::param::get("use_dist_estimates", use_dist_estimates);


    ros::param::get("W_Wn_factor", nmpc_struct.W_Wn_factor);
    int u_idx = 0;
    ros::param::get("F_x_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("F_y_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("F_z_ref", nmpc_struct.U_ref(u_idx++));
    ros::param::get("Mz_ref", nmpc_struct.U_ref(u_idx++));

    assert(u_idx == NMPC_NU);

    int w_idx = 0;
    ros::param::get("W_x", nmpc_struct.W(w_idx++));
    ros::param::get("W_y", nmpc_struct.W(w_idx++));
    ros::param::get("W_z", nmpc_struct.W(w_idx++));
    ros::param::get("W_u", nmpc_struct.W(w_idx++));
    ros::param::get("W_v", nmpc_struct.W(w_idx++));
    ros::param::get("W_w", nmpc_struct.W(w_idx++));
    ros::param::get("W_psi", nmpc_struct.W(w_idx++));
    ros::param::get("W_r", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fx", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fy", nmpc_struct.W(w_idx++));
    ros::param::get("W_Fz", nmpc_struct.W(w_idx++));
    ros::param::get("W_Mz", nmpc_struct.W(w_idx++));
    assert(w_idx == NMPC_NY);

    nmpc_struct.sample_time = sampleTime;

    NMPC_PC* nmpc_pc = new NMPC_PC(nmpc_struct);
    ros::Rate rate(1 / sampleTime);

    current_pos_att.resize(6);
    current_vel_rate.resize(6);
    current_vel_body.resize(6);
    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);

    ref_traj_type = 0;
    ref_position << 0, 0, 0;
    ref_velocity << 0, 0, 0;
    
    control_stop = false;

    for (int i = 0; i < (int)(1 / sampleTime); ++i)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && !control_stop)
    {
        t = ros::Time::now().toSec();

        if (current_state_msg.mode != "OFFBOARD" && print_flag_offboard == 1)
        {
            ROS_INFO("OFFBOARD mode is not enabled!");
            print_flag_offboard = 0;
        }
        if (!current_state_msg.armed && print_flag_arm == 1)
        {
            ROS_INFO("Vehicle is not armed!");
            print_flag_arm = 2;
        }
        else if (current_state_msg.armed && print_flag_arm == 2)
        {
            ROS_INFO("Vehicle is armed!");
            print_flag_arm = 0;
        }

        if (current_state_msg.mode == "ALTCTL")
        {
            pos_ref = current_pos_att;
            if (print_flag_altctl == 1)
            {
                ROS_INFO("ALTCTL mode is enabled!");
                print_flag_altctl = 0;
            }
        }

        if (!nmpc_pc->return_control_init_value())
        {
            nmpc_pc->nmpc_init(pos_ref, nmpc_pc->nmpc_struct);
            if (nmpc_struct.verbose && nmpc_pc->return_control_init_value())
            {
                std::cout << "***********************************\n";
                std::cout << "NMPC: initialized correctly\n";
                std::cout << "***********************************\n";
            }
        }

        while (ros::ok() && !control_stop)
        {

            t_cc_loop = ros::Time::now().toSec() - t;
            if (std::fmod(std::abs(t_cc_loop - (int)(t_cc_loop)), (double)(sampleTime)) == 0)
                std::cout << "loop time for outer NMPC: " << t_cc_loop << " (sec)"
                          << "\n";

            // Setting up state-feedback [x,y,z,u,v,w,psi,r]
            current_states = {current_pos_att.at(0),
                              current_pos_att.at(1),
                              current_pos_att.at(2),
                              current_vel_rate.at(0),
                              current_vel_rate.at(1),
                              current_vel_rate.at(2),
                              current_pos_att.at(5),
                              current_vel_rate.at(5)
                              };

            ref_trajectory = {ref_position(0),
                              ref_position(1),
                              -ref_position(2),
                              0.0,
                              0.0,
                              0.0,
                              ref_yaw_rad,
                              0.0
                              };                  




            std::cout << "current_states = ";
            for (int idx = 0; idx < current_states.size(); idx++)
            {
                std::cout << current_states[idx] << ",";
            }
            std::cout << "\n";

            std::cout << "ref_trajectory = ";
            for (int idx = 0; idx < ref_trajectory.size(); idx++)
            {
                std::cout << ref_trajectory[idx] << ",";
            }
            std::cout << "\n";





           online_data.distFx = dist_Fx.data;
           online_data.distFy = dist_Fy.data;
           online_data.distFz = dist_Fz.data;


           //online_data.distFx = 0.0;
           //online_data.distFy = 0.0;
           //online_data.distFz = 0.0;


            nmpc_pc->nmpc_core(nmpc_struct,
                               nmpc_pc->nmpc_struct,
                               nmpc_pc->nmpc_cmd_struct,
                               ref_trajectory,
                               online_data,
                               current_states);

            if (nmpc_pc->acado_feedbackStep_fb != 0)
                control_stop = true;

            if (std::isnan(nmpc_pc->nmpc_struct.u[0]) == true || std::isnan(nmpc_pc->nmpc_struct.u[1]) == true ||
                std::isnan(nmpc_pc->nmpc_struct.u[2]) == true || std::isnan(nmpc_pc->nmpc_struct.u[3]) == true)
            {
                ROS_ERROR_STREAM("Controller ERROR at time = " << ros::Time::now().toSec() - t << " (sec)");
                control_stop = true;
                exit(0);
            }

            nmpc_pc->publish_rpyFz(nmpc_pc->nmpc_cmd_struct);

            print_flag_offboard = 1;
            print_flag_arm = 1;
            print_flag_altctl = 1;

            ros::spinOnce();
            rate.sleep();
        }

        nmpc_pc->publish_rpyFz(nmpc_pc->nmpc_cmd_struct);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
