#include "heron_controller/controller.h"

Controller::Controller(ros::NodeHandle &n):node_(n) {
    force_compensator_ = new ForceCompensator(node_);

    //Assume no messages are being received. Don't send out anything new until commands are received
    control_mode = NO_CONTROL;

    ros::NodeHandle prv_node_("~");

    active_control_srv = node_.advertiseService("activate_control", &Controller::activate_control_service, this);
    is_active_control = true;

    //Timeouts for sensors
    //if no data has been received in a while, disable certain PID controls
    prv_node_.param<double>("imu_data_timeout", imu_data_timeout_, 1/5.0);
    imu_data_time_ = 0;
    imu_timeout_ = true;

    prv_node_.param<double>("vel_data_timeout", vel_data_timeout_, 1/5.0);
    vel_data_time_ = 0;
    vel_timeout_ = true;

    twist_cmd_time_ = 0;
    course_cmd_time_ = 0;
    helm_cmd_time_ = 0;
    wrench_cmd_time_ = 0;

    //Setup Fwd Vel Controller
    fvel_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("fwd_vel_debug",1000);
    prv_node_.param<double>("fwd_vel/kf", fvel_kf_,10); //Feedforward Gain
    prv_node_.param<double>("fwd_vel/kp", fvel_kp_,90.0);  //Proportional Gain
    prv_node_.param<double>("fwd_vel/kd", fvel_kd_,1.0); //Derivative Gain
    prv_node_.param<double>("fwd_vel/ki", fvel_ki_,0.0); //Integral Gain
    prv_node_.param<double>("fwd_vel/imax", fvel_imax_,0.0); //Clamp Integral Outputs
    prv_node_.param<double>("fwd_vel/imin", fvel_imin_,0.0);
    fvel_meas_ = 0;

    //Setup Yaw Rate Controller
    yr_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("yaw_rate_debug",1000);
    prv_node_.param<double>("yaw_rate/kf", yr_kf_,10); //Feedforward Gain
    prv_node_.param<double>("yaw_rate/kp", yr_kp_,2.0);  //Proportional Gain
    prv_node_.param<double>("yaw_rate/kd", yr_kd_,1.0); //Derivative Gain
    prv_node_.param<double>("yaw_rate/ki", yr_ki_,0.0); //Integral Gain
    prv_node_.param<double>("yaw_rate/imax", yr_imax_,0.0); //Clamp Integral Outputs
    prv_node_.param<double>("yaw_rate/imin", yr_imin_,0.0);
    yr_meas_ = 0;

    //Setup Yaw Controller
    y_dbg_pub_ = node_.advertise<geometry_msgs::Vector3>("yaw_debug",1000);

    prv_node_.param<double>("yaw/kp", y_kf_,5.0);
    prv_node_.param<double>("yaw/kp", y_kp_,5.0);
    prv_node_.param<double>("yaw/kd", y_kd_,1.0);
    prv_node_.param<double>("yaw/ki", y_ki_,0.5);
    prv_node_.param<double>("yaw/imax", y_imax_,0.0); //clamp integral output at max yaw yorque
    prv_node_.param<double>("yaw/imin", y_imin_,0.0);
    y_meas_ = 0;

    ROS_DEBUG("Fwd Vel Params (F,P,I,D,iMax,iMin):%f,%f,%f,%f,%f,%f",fvel_kf_,fvel_kp_, fvel_ki_,fvel_kd_,fvel_imax_,fvel_imin_);
    ROS_DEBUG("Yaw Rate Params (F,P,I,D,iMax,iMin):%f,%f,%f,%f,%f,%f",yr_kf_,yr_kp_, yr_ki_,yr_kd_,yr_imax_,yr_imin_);
    ROS_DEBUG("Yaw Params (F,P,I,D,iMax,iMin):%f,%f,%f,%f,%f,%f",y_kf_,y_kp_, y_ki_,y_kd_,y_imax_,y_imin_);


    fvel_pid_.reset();
    fvel_pid_.initPid(fvel_kp_,fvel_ki_,fvel_kd_,fvel_imax_,fvel_imin_);
    fvel_cmd_ = 0;

    yr_pid_.reset();
    yr_pid_.initPid(yr_kp_,yr_ki_,yr_kd_,yr_imax_,yr_imin_);
    yr_cmd_ = 0;

    //Setup Yaw Controller
    y_pid_.reset();
    y_pid_.initPid(y_kp_,y_ki_,y_kd_,y_imax_,y_imin_);
    y_cmd_ = 0;

    prv_node_.param<double>("max/fwd_vel", max_fwd_vel_,MAX_FWD_VEL);
    prv_node_.param<double>("max/fwd_force", max_fwd_force_,2*MAX_FWD_THRUST); //2 thrusters
    prv_node_.param<double>("max/bck_vel",max_bck_vel_,MAX_BCK_VEL);
    prv_node_.param<double>("max/bck_force",max_bck_force_,2*MAX_BCK_THRUST);

    prv_node_.param<double>("cov_limits/velocity", vel_cov_limit_, 0.28);
    prv_node_.param<double>("cov_limits/imu", imu_cov_limit_, 1.0);
}

double Controller::fvel_compensator() {
  //calculate pid force X
  double fvel_error = fvel_cmd_ - fvel_meas_;
  double fvel_comp_output = fvel_pid_.computeCommand(fvel_error, ros::Duration(1/20.0));
  fvel_comp_output = fvel_comp_output + fvel_kf_*fvel_cmd_;

  geometry_msgs::Vector3 dbg_info;
  dbg_info.x = fvel_cmd_;
  dbg_info.y = fvel_meas_;
  dbg_info.z = fvel_comp_output;
  fvel_dbg_pub_.publish(dbg_info);

  return fvel_comp_output;
}

double Controller::yr_compensator() {
    //calculate pid torque z
    double yr_error = yr_cmd_ - yr_meas_;
    double yr_comp_output = yr_pid_.computeCommand(yr_error, ros::Duration(1/20.0));
    yr_comp_output = yr_comp_output + yr_kf_*yr_cmd_; //feedforward

    geometry_msgs::Vector3 dbg_info;
    dbg_info.x = yr_cmd_;
    dbg_info.y = yr_meas_;
    dbg_info.z = yr_comp_output;
    yr_dbg_pub_.publish(dbg_info);

    return yr_comp_output;
}

double Controller::y_compensator() {
    //calculate pid torque z

    if (y_meas_ < 0)
        y_meas_ = y_meas_ + 2*PI;

    double y_error = y_cmd_ - y_meas_;

    if (fabs(y_error) > PI){
        if (y_cmd_ > PI) //presumably y_meas_ < PI
            y_error =-( y_meas_ + (2*PI - y_cmd_));
        else //y_cmd_ < pi, y_meas > pi
            y_error =  y_cmd_ +  (2*PI - y_meas_);
    }

    double y_comp_output = y_pid_.computeCommand(y_error, ros::Duration(1/20.0));

    geometry_msgs::Vector3 dbg_info;
    dbg_info.x = y_cmd_;
    dbg_info.y = y_meas_;
    dbg_info.z = y_comp_output;
    y_dbg_pub_.publish(dbg_info);

    return y_comp_output;
}

double deadzone_force(double force, double pos_limit, double neg_limit) {
  if (force > 0) {
    if (force < pos_limit) {
        return 0;
    }//if
  } else {
    if (force > -neg_limit) {
        return 0;
    }//if
  }//else

  return force;
}

void Controller::fwd_vel_mapping() {
  if (fvel_cmd_ > 0) {
    force_output_.force.x = deadzone_force(max_fwd_force_ * fvel_cmd_ / max_fwd_vel_, max_fwd_force_ * 0.06, max_bck_force_ * 0.06);
  } else {
      force_output_.force.x = deadzone_force(max_bck_force_ * fvel_cmd_ / max_bck_vel_, max_fwd_force_ * 0.06, max_bck_force_ * 0.06);
  }//else
}//fwd_vel_mapping

void Controller::update_fwd_vel_control() {
  force_output_.force.x = deadzone_force(fvel_compensator(), max_fwd_force_ * 0.06, max_bck_force_ * 0.06);
}

void Controller::update_yaw_rate_control() {
  force_output_.torque.z = deadzone_force(yr_compensator(), 2, 2);
}

void Controller::update_yaw_control() {
  yr_cmd_ = y_compensator();
  force_output_.torque.z = deadzone_force(yr_compensator(), 2, 2);
}

//Callback to receive twist msgs (cmd_vel style)
void Controller::twist_callback(const geometry_msgs::Twist msg) {
  yr_cmd_ = msg.angular.z;
  update_yaw_rate_control();

  fvel_cmd_ = msg.linear.x;

  if (control_mode == TWIST_LIN_CONTROL) {
    fwd_vel_mapping();
  } else {
    update_fwd_vel_control();
  }

  twist_cmd_time_ = ros::Time::now().toSec();
}//twist_callback


//Callback to receive raw wrench commands (force along x axis and torque about z axis).
void Controller::wrench_callback(const geometry_msgs::Wrench msg) {
    force_output_.force.x = msg.force.x;
    force_output_.torque.z = msg.torque.z;
    wrench_cmd_time_ = ros::Time::now().toSec();
}

//Callback for yaw command which receives a yaw (rad) and speed (m/s) command
void Controller::course_callback(const heron_msgs::Course msg) {
    //Save Yaw Command and process it
    y_cmd_ = msg.yaw;
    update_yaw_control();

    // Calculate speed command
    fvel_cmd_ = msg.speed;
    update_fwd_vel_control();

    course_cmd_time_ = ros::Time::now().toSec();
}

//Callback for helm commands which receives a thrust percentage (0..1) and a yaw rate (rad/s)
void Controller::helm_callback(const heron_msgs::Helm msg) {
    //Basic Helm Control

    //Calculate Thrust control
    double thrust = msg.thrust;
    if (thrust >= 0)
        force_output_.force.x = thrust * (max_fwd_force_/1);
    else
        force_output_.force.x = thrust * (max_bck_force_/1);

    //Save yaw rate command to be processed when feedback is available
    yr_cmd_ = msg.yaw_rate;
    update_yaw_rate_control();

    helm_cmd_time_ = ros::Time::now().toSec();
}

//ENU
void Controller::odom_callback(const nav_msgs::Odometry msg) {

    //check if navsat/vel is being integrated into odometry
    if (msg.twist.covariance[0] < vel_cov_limit_ && msg.twist.covariance[7] < vel_cov_limit_) {
      vel_data_time_ = ros::Time::now().toSec();
    }//if

    //check if imu/data is being integrated into odometry
    if (msg.pose.covariance[35] < imu_cov_limit_ && msg.twist.covariance[35] < imu_cov_limit_) {
      imu_data_time_ = ros::Time::now().toSec();
    }//if

    y_meas_ = tf::getYaw(msg.pose.pose.orientation);
    yr_meas_ = msg.twist.twist.angular.z;
    fvel_meas_ = msg.twist.twist.linear.x * std::cos(y_meas_) + msg.twist.twist.linear.y * std::sin(y_meas_);

    switch (control_mode) {
        case COURSE_CONTROL:
          update_fwd_vel_control();
          update_yaw_control();
          break;
        case HELM_CONTROL:
          update_yaw_rate_control();
          break;
        case WRENCH_CONTROL:
          break;
        case TWIST_CONTROL:
          update_fwd_vel_control();
          update_yaw_rate_control();
          break;
        case TWIST_LIN_CONTROL:
          update_yaw_rate_control();
          break;
        case NO_CONTROL:
          break;
    }//switch
}//odom_callback


void Controller::console_update(const ros::TimerEvent& event) {

    std::string output="";
    switch (control_mode)
    {
        case COURSE_CONTROL:
            output = "Boat controlling yaw position";
            break;
        case HELM_CONTROL:
            output = "Boat Controlling yaw rate";
            break;
        case WRENCH_CONTROL:
            output = "Boat in raw wrench/RC control";
            break;
        case TWIST_CONTROL:
            output = "Boat controlling forward and yaw velocity";
            break;
        case TWIST_LIN_CONTROL:
            output = "Boat controlling yaw velocity and mapping velocity linearly";
            break;
        case NO_CONTROL:
            output = "No commands being processed";
            break;
        default:
            break;
     }

     if (imu_timeout_)
        output+=": IMU data not received or being received too slowly";

     if (vel_timeout_)
        output+=": GPS Velocity data not received or being received too slowly";

     ROS_INFO("%s",output.c_str());
}

void Controller::control_update(const ros::TimerEvent& event) {

    if (ros::Time::now().toSec() - imu_data_time_ > imu_data_timeout_) {
        imu_timeout_ = true;
    } else {
        imu_timeout_ = false;
    }//else

    if (ros::Time::now().toSec() - vel_data_time_ > vel_data_timeout_) {
        vel_timeout_ = true;
    } else {
        vel_timeout_ = false;
    }//else

    std::vector<double> find_latest;

    if (!imu_timeout_) {
      find_latest.push_back(twist_cmd_time_);
      find_latest.push_back(helm_cmd_time_);
    }//if

    if (!imu_timeout_ && !vel_timeout_) {
        find_latest.push_back(course_cmd_time_);
    }//if

    find_latest.push_back(wrench_cmd_time_);
    double max = *std::max_element(find_latest.begin(), find_latest.end());

    if (max == 0) {
      control_mode=NO_CONTROL;
      force_output_.torque.z = 0;
      force_output_.force.x = 0;
      return;
    } else if (max == twist_cmd_time_ && !imu_timeout_ && !vel_timeout_) {
      control_mode=TWIST_CONTROL;
    } else if (max == twist_cmd_time_ && !imu_timeout_) {
      control_mode=TWIST_LIN_CONTROL;
    } else if (max == course_cmd_time_ && !imu_timeout_ && !vel_timeout_) {
      control_mode=COURSE_CONTROL;
    } else if(max == helm_cmd_time_ && !imu_timeout_) {
      control_mode=HELM_CONTROL;
    } else if(max == wrench_cmd_time_) {
      control_mode=WRENCH_CONTROL;
    }//elseif

    if (!is_active_control) {
        return;
    }//if


    force_compensator_->pub_thrust_cmd(force_output_);
}

bool Controller::activate_control_service(heron_controller::ActivateControl::Request& req, heron_controller::ActivateControl::Response& resp) {
  is_active_control = req.set_active;
  resp.is_active = is_active_control;
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "controller");
    ros::NodeHandle nh;
    Controller kf_control(nh);

    ros::Subscriber twist_sub = nh.subscribe("cmd_vel",1,&Controller::twist_callback, &kf_control);
    ros::Subscriber wrench_sub = nh.subscribe("cmd_wrench",1, &Controller::wrench_callback, &kf_control);
    ros::Subscriber helm_sub = nh.subscribe("cmd_helm",1, &Controller::helm_callback,&kf_control);
    ros::Subscriber course_sub = nh.subscribe("cmd_course",1, &Controller::course_callback,&kf_control);

    ros::Subscriber odom_sub = nh.subscribe("odometry/filtered",1, &Controller::odom_callback, &kf_control);

    ros::Timer control_output = nh.createTimer(ros::Duration(1/50.0), &Controller::control_update,&kf_control);
    ros::Timer console_update = nh.createTimer(ros::Duration(1), &Controller::console_update, &kf_control);

    ros::spin();

    return 0;
}
