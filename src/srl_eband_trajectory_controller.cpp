/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Luigi Palmieri
*********************************************************************/

#include <srl_eband_local_planner/srl_eband_trajectory_controller.h>
#include <tf/transform_datatypes.h>
/// these are platform dependent, we could set them as params
#define TRANS_VEL_ABS_LIMIT  1.0
#define ROT_VEL_ABS_LIMIT 1.57;
tf::TransformListener* tf_listener;

namespace srl_eband_local_planner{

using std::min;
using std::max;


SrlEBandTrajectoryCtrl::SrlEBandTrajectoryCtrl() : costmap_ros_(NULL), initialized_(false), band_set_(false), visualization_(false) {}


SrlEBandTrajectoryCtrl::SrlEBandTrajectoryCtrl(std::string name, costmap_2d::Costmap2DROS* costmap_ros, tf::TransformListener* tf)
  : costmap_ros_(NULL), initialized_(false), band_set_(false), visualization_(false)
{
  controller_frequency_ = 6.66;
  curvature_guarding_thrs_ = 0.65;
  warning_robot_radius_ = 2.0;
  min_vel_limited_curvature_ = 0.35;
  front_laser_frame_ = "laser_front_link";
  rear_laser_frame_ = "laser_rear_link";
  num_points_front_robot_ = 0;
  num_points_rear_robot_ = 0;
  front_laser_topic_ = "/spencer/sensors/laser_front/echo0";
  rear_laser_topic_ = "/spencer/sensors/laser_rear/echo0";
  local_path_topic_ = "/move_base_node/HANPLocalPlanner/local_plan";
  backward_motion_on_ - true;
  robot_frame_ = "base_link_flipped";
  limit_vel_based_laser_points_density_= true;
  max_translational_vel_due_to_laser_points_density_ = 0.5;
  warning_robot_angle_ = M_PI/2;
  max_rotational_velocity_turning_on_spot_ = 0.75;
  start_to_stop_goal_ = 2.25;
  human_legibility_on_ = true;
  circumscribed_radius_ = 0.67;
  // initialize planner
  initialize(name, costmap_ros, tf);
  compute_curvature_properties_ = new CurvatureProperties();
  // tf_listener = new tf::TransformListener();
  // Initialize pid object (note we'll be further clamping its output)
  pid_.initPid(1, 0, 0, 10, -10);

}


SrlEBandTrajectoryCtrl::~SrlEBandTrajectoryCtrl() {


}

/// ==================================================================================
/// setCostMap()
/// ==================================================================================
void SrlEBandTrajectoryCtrl::setCostMap(costmap_2d::Costmap2DROS* costmap_ros){

  // copy adress of costmap (handed over from move_base via eband wrapper)
  costmap_ros_ = costmap_ros;
  circumscribed_radius_ = getCircumscribedRadius(*costmap_ros_);
  return;
}

void SrlEBandTrajectoryCtrl::setDifferentialDriveVelLimits(double v, double w){

      ROS_DEBUG_NAMED("Eband_HRI","Setting the max Velocities (%f, %f)", v, w);
      max_vel_lin_hri_ = v;
      return;
}

/// =======================================================================================
/// callbackDynamicReconfigure
/// =======================================================================================
void SrlEBandTrajectoryCtrl::callbackDynamicReconfigure(srl_eband_local_planner::srlEBandLocalPlannerConfig &config, uint32_t level ){

  ROS_DEBUG("Reconfiguring Eband Tracker");

  tolerance_trans_ = config.xy_goal_tolerance_dyn;
  tolerance_rot_ = config.yaw_goal_tolerance_dyn;
  rot_stopping_turn_on_the_spot_ = config.rot_stopping_turn_on_the_spot_dyn;
  max_vel_lin_ = config.max_vel_lin_dyn;
  min_vel_lin_ = config.min_vel_lin_dyn;
  acc_max_ = config.max_acceleration_dyn;
  min_vel_th_ = config.min_vel_th_dyn;
  max_vel_th_ = config.max_vel_th_dyn;
  min_in_place_vel_th_ = config.min_in_place_vel_th_dyn;
  in_place_trans_vel_ = config.in_place_trans_vel_dyn;
  k_p_ = config.k_prop_dyn;
  k_nu_ = config.k_damp_dyn;
  ts_ = config.Ts_dyn;
  k_one_ = config.Kv_one_dyn;
  bubble_velocity_multiplier_ = config.Vel_gain_dyn;
  tracker_on_ = config.tracker_on;
  curvature_guarding_thrs_ = config.curvature_guarding_thrs;
  k_two_ = config.Kv_two_dyn;
  backward_motion_on_ = config.backward_motion_on_dyn;
  b_ = config.B_dyn;
  smoothed_eband_ = config.smoothed_eband_dyn;
  acc_max_trans_ = config.max_translational_acceleration_dyn;
  acc_max_rot_ = config.max_rotational_acceleration_dyn;
  limit_vel_based_on_curvature_ = config.limit_vel_based_on_curvature;
  min_vel_limited_curvature_ = config.min_vel_limited_curvature;
  limit_vel_based_laser_points_density_ = config.limit_vel_based_laser_points_density_dyn;
  max_translational_vel_due_to_laser_points_density_ = config.max_translational_vel_due_to_laser_points_density_dyn;
  warning_robot_angle_ = config.warning_robot_angle_dyn;
  warning_robot_radius_ = config.warning_robot_radius_dyn;
  max_rotational_velocity_turning_on_spot_ = config.max_rotational_velocity_turning_on_spot_dyn;
  human_legibility_on_ = config.human_legibility_on_dyn;
  trans_vel_goal_ = config.trans_vel_goal_dyn;
  start_to_stop_goal_ = config.start_to_stop_goal_dyn;
  context_cost_function_->setParams(config.cc_alpha_max, config.cc_d_low,
              config.cc_d_high, config.cc_beta, config.cc_min_scale,
              config.sim_time, config.publish_predictions, config.publish_curr_traj);

  limit_vel_based_on_hri_ = config.limit_vel_based_on_hri;
  if(backward_motion_on_){
    robot_frame_ = "base_link_flipped";
  }else{
    robot_frame_ = "base_link";
  }

  ROS_DEBUG("New max_vel_lin %f", max_vel_lin_);
  ROS_DEBUG("New min_vel_lin_ %f", min_vel_lin_);
  ROS_DEBUG("Velocity Gain changed to %f", bubble_velocity_multiplier_);
  ROS_DEBUG("warning_robot_angle_ to %f", warning_robot_angle_);
  ROS_DEBUG("New max_vel_th %f", max_vel_th_);

  return;

}

/// =======================================================================================
/// initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
/// =======================================================================================
void SrlEBandTrajectoryCtrl::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros, tf::TransformListener* tf)
{

  // check if trajectory controller is already initialized
  if(!initialized_)
  {
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle node_private("~/" + name);
    trans_vel_goal_ = 0.5;
    start_to_stop_goal_ = 2.25;
    max_vel_th_hri_ = 1.57;
    max_vel_lin_hri_ = 1.3;
    // read parameters from parameter server
    node_private.param("max_vel_lin", max_vel_lin_, 1.0);
    node_private.param("max_vel_th", max_vel_th_, 1.57);
    node_private.param("min_vel_lin", min_vel_lin_, 0.1);
    node_private.param("min_vel_th", min_vel_th_, 0.0);
    node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);
    node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);
    node_private.param("rot_stopping_turn_on_the_spot", rot_stopping_turn_on_the_spot_, 0.05);
    node_private.param("xy_goal_tolerance", tolerance_trans_, 0.35);
    node_private.param("yaw_goal_tolerance", tolerance_rot_, 0.04);
    node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);
    node_private.param("k_prop", k_p_, 4.0);
    node_private.param("k_damp", k_nu_, 3.5);
    node_private.param("max_acceleration", acc_max_, 0.5);
    node_private.param("virtual_mass", virt_mass_, 0.75);
    node_private.param("max_translational_acceleration", acc_max_trans_, 1.0);
    node_private.param("max_rotational_acceleration", acc_max_rot_, 1.5);
    node_private.param("rotation_correction_threshold", rotation_correction_threshold_, 0.5);
    // diffferential drive parameters
    node_private.param("differential_drive", differential_drive_on_, true);
    node_private.param("k_int", k_int_, 0.005);
    node_private.param("k_diff", k_diff_, -0.005);
    node_private.param("bubble_velocity_multiplier", bubble_velocity_multiplier_, 3.0);
    node_private.param("rotation_threshold_multiplier", rotation_threshold_multiplier_, 1.0); //0.75);
    node_private.param("disallow_hysteresis", disallow_hysteresis_, true); //0.75);
    node_private.param("Ts", ts_ , 0.1);
    node_private.param("Kv_one", k_one_, 1.0);
    node_private.param("Kv_two", k_two_, 1.0);
    node_private.param("B", b_, 0.15);
    node_private.param("smoothed_eband", smoothed_eband_, true);
    node_private.param("lookahed", lookahed_, 2);
    node_private.getParam("/move_base_node/controller_frequency", this->controller_frequency_);
    node_private.getParam("limit_vel_based_laser_points_density", this->limit_vel_based_laser_points_density_);
    node_private.getParam("front_laser_frame", this->front_laser_frame_);
    node_private.getParam("rear_laser_frame", this->rear_laser_frame_);
    node_private.getParam("warning_robot_radius", this->warning_robot_radius_);
    node_private.getParam("front_laser_topic", this->front_laser_topic_);
    node_private.getParam("rear_laser_topic", this->rear_laser_topic_);
    node_private.getParam("backward_motion_on", this->backward_motion_on_);
    node_private.getParam("local_path_topic", this->local_path_topic_);
    node_private.getParam("max_translational_vel_due_to_laser_points_density", this->max_translational_vel_due_to_laser_points_density_);
    node_private.getParam("trans_vel_goal", trans_vel_goal_);
    node_private.getParam("start_to_stop_goal",start_to_stop_goal_);
    /// define subscribers
    sub_front_laser_ =  node_private.subscribe(front_laser_topic_, 1, &SrlEBandTrajectoryCtrl::callbackLaserScanReceived, this);
    sub_rear_laser_  =  node_private.subscribe(rear_laser_topic_, 1, &SrlEBandTrajectoryCtrl::callbackLaserScanReceived, this);
    pub_local_path_ = node_private.advertise<nav_msgs::Path>(local_path_topic_, 1);

    // Ctrl_rate, k_prop, max_vel_lin, max_vel_th, tolerance_trans, tolerance_rot, min_in_place_vel_th
    in_final_goal_turn_ = false;

    // copy adress of costmap and Transform Listener (handed over from move_base)
    costmap_ros_ = costmap_ros;

    tf_listener = tf;

    planner_frame_ = "odom";
    // init velocity for interpolation
    last_vel_.linear.x = 0.0;
    last_vel_.linear.y = 0.0;
    last_vel_.linear.z = 0.0;
    last_vel_.angular.x = 0.0;
    last_vel_.angular.y = 0.0;
    last_vel_.angular.z = 0.0;

    // set the general refernce frame to that in which the band is given
    geometry_msgs::Pose2D tmp_pose2D;
    tmp_pose2D.x = 0.0;
    tmp_pose2D.y = 0.0;
    tmp_pose2D.theta = 0.0;
    Pose2DToPose(ref_frame_band_, tmp_pose2D);

    // set initialized flag
    initialized_ = true;


    initial_turn_ = true;
    initial_band_ = false;

    x_initial_band_ = 0;
    y_initial_band_ = 0;
    theta_initial_band_ = 0;

    tracker_on_ = false;
    path_id_ = 0;
    limit_vel_based_on_curvature_ = true;

    context_cost_function_ =  new hanp_local_planner::ContextCostFunction();
    context_cost_function_->initialize(planner_frame_, tf_listener);

    circumscribed_radius_ = getCircumscribedRadius(*costmap_ros_);

  }
  else
  {
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
}


/// =======================================================================================
/// publishLocalPlan
/// =======================================================================================
void SrlEBandTrajectoryCtrl::publishLocalPlan(base_local_planner::Trajectory local_traj){

  int n_traj_points = (int)local_traj.getPointsSize();
  nav_msgs::Path path_to_publish;
  path_to_publish.header.frame_id = planner_frame_;
  path_to_publish.header.stamp = ros::Time();
  path_to_publish.header.seq = path_id_++;

  path_to_publish.poses.resize(n_traj_points);

  for (int i=0; i<n_traj_points; i++){
    double xi=0;
    double yi=0;
    double thi=0;
    local_traj.getPoint(i, xi, yi, thi);
    path_to_publish.poses[i].header.frame_id = planner_frame_;
    path_to_publish.poses[i].header.stamp = ros::Time();
    path_to_publish.poses[i].header.seq = i;
    path_to_publish.poses[i].pose.position.x = xi;
    path_to_publish.poses[i].pose.position.y = yi;
    path_to_publish.poses[i].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,thi);
  }

  pub_local_path_.publish(path_to_publish);
  ROS_DEBUG("Local Path published to %s with %d points", local_path_topic_.c_str(), n_traj_points);
  return;
}


/// =======================================================================================
/// callbackLaserScanReceived(const sensor_msgs::LaserScan& laserscan), read laser scan
/// =======================================================================================
void SrlEBandTrajectoryCtrl::callbackLaserScanReceived(const sensor_msgs::LaserScan& laserscan){

  bool front_laser = false;
  bool rear_laser = false;

  if(strcmp(front_laser_frame_.c_str(), laserscan.header.frame_id.c_str() ) == 0){
    front_laser = true;
    if(backward_motion_on_)
      {
        num_points_rear_robot_ = 0;
      }
    else
      {
        num_points_front_robot_ = 0;
      }
  }

  if(strcmp(rear_laser_frame_.c_str(), laserscan.header.frame_id.c_str() ) == 0){
    rear_laser = true;
    if(backward_motion_on_)
        {
          num_points_front_robot_ = 0;
        }
    else
        {
          num_points_rear_robot_ = 0;
        }
  }


  for(size_t p_i = 0; p_i < laserscan.ranges.size(); p_i++) {

      const double phi = laserscan.angle_min + laserscan.angle_increment * p_i;
      const double rho = laserscan.ranges[p_i];

      // Only process points with valid range data
      const bool is_in_range = rho > laserscan.range_min && rho < laserscan.range_max;


      if(is_in_range)
      {
          // Assuming that laser frame is almost equal to the base_link frame,
          // to reduce number of transforms

          // Check if inside warning radius
          if(rho < warning_robot_radius_ && fabs(phi) < warning_robot_angle_ ){
              //ROS_DEBUG("Value of Phi %f and Rho %f", phi, rho);
              if(front_laser)
                {
                  if(backward_motion_on_)
                    {
                      num_points_rear_robot_++;
                    }
                  else
                    {
                      num_points_front_robot_++;
                    }
                }
                else if(rear_laser){

                    if(backward_motion_on_)
                        {
                          num_points_front_robot_++;
                        }
                    else
                        {
                          num_points_rear_robot_++;
                        }

                }
          }

      }
  }

  ROS_DEBUG("Eband_collision_error","Points in front of the robot %d, points rear side of the robot %d",num_points_front_robot_, num_points_rear_robot_ );
  return ;
}


/// =======================================================================================
/// setVisualization(boost::shared_ptr<SrlEBandVisualization> target_visual)
/// =======================================================================================
void SrlEBandTrajectoryCtrl::setVisualization(boost::shared_ptr<SrlEBandVisualization> target_visual)
{
  target_visual_ = target_visual;
  visualization_ = true;
}


/// =======================================================================================
/// setBand(const std::vector<Bubble>& elastic_band)
/// =======================================================================================
bool SrlEBandTrajectoryCtrl::setBand(const std::vector<Bubble>& elastic_band)
{
  elastic_band_ = elastic_band;
  band_set_ = true;
  return true;
  previous_angular_error_=0;
  integral_angular_=0;

  if(tracker_on_){
      // create local variables
      std::vector<geometry_msgs::PoseStamped> tmp_plan;
      // adapt plan to band
      tmp_plan.clear();
      plan_.clear();
      double old_x = 0;
      double old_y = 0;
      ROS_INFO("Elastic band received in frame %s", elastic_band_[0].center.header.frame_id.c_str());

      for(int i = 0; i < ((int) elastic_band_.size()); i++)
      {
        if(i>0){
          double t = 0;
          while( t < 1 ){
              geometry_msgs::PoseStamped p;
              p.header = elastic_band_[i].center.header;
              p.pose.position.x =  t*(elastic_band_[i].center.pose.position.x - old_x) + old_x;
              p.pose.position.y =  t*(elastic_band_[i].center.pose.position.y - old_y) + old_y;
              p.pose.orientation = elastic_band_[i].center.pose.orientation;
              t=t+0.5;
              geometry_msgs::PoseStamped pt = transformPose(p);
              tmp_plan.push_back( pt );
              ROS_DEBUG("Point added %f %f to the path from the elastic band", p.pose.position.x, p.pose.position.y);
              ROS_DEBUG("Related to the elastic band point %f, %f", elastic_band_[i].center.pose.position.x,
                          elastic_band_[i].center.pose.position.y);
          }
        }else{
          geometry_msgs::PoseStamped p;
          p.header = elastic_band_[i].center.header;
          p.pose.position.x =  elastic_band_[i].center.pose.position.x;
          p.pose.position.y =  elastic_band_[i].center.pose.position.y;
          p.pose.orientation = elastic_band_[i].center.pose.orientation;
          geometry_msgs::PoseStamped pt = transformPose(p);
          // set centers of bubbles to StampedPose in plan
          tmp_plan.push_back( pt );
          ROS_DEBUG("Initial point added %f %f to the path from the elastic band", elastic_band_[i].center.pose.position.x, elastic_band_[i].center.pose.position.y);
        }
        old_x = elastic_band_[i].center.pose.position.x;
        old_y = elastic_band_[i].center.pose.position.y;
      }
      //write to referenced variable and done
      plan_ = tmp_plan;
      curr_control_index_ = 0;
      /// setting Feedforward commands
      dx_d_.resize(plan_.size());
      dy_d_.resize(plan_.size());
      for(size_t i=0; i<dx_d_.size()-1; i++){
              dx_d_[i]=(plan_[i+1].pose.position.x-plan_[i].pose.position.x)/ts_;
              dy_d_[i]=(plan_[i+1].pose.position.y-plan_[i].pose.position.y)/ts_;
              if(dx_d_[i]==0)
                  dx_d_[i]=0.01;
              if(dy_d_[i]==0)
                  dy_d_[i]=0.01;
      }

  }

  ROS_DEBUG("Setting Band ended");

  return true;
}

/// =======================================================================================
/// setOdometry(const nav_msgs::Odometry& odometry)
/// =======================================================================================
bool SrlEBandTrajectoryCtrl::setOdometry(const nav_msgs::Odometry& odometry)
{
  odom_vel_.linear.x = odometry.twist.twist.linear.x;
  odom_vel_.linear.y = odometry.twist.twist.linear.y;
  odom_vel_.linear.z = 0.0;
  odom_vel_.angular.x = 0.0;
  odom_vel_.angular.y = 0.0;
  odom_vel_.angular.z = odometry.twist.twist.angular.z;

  x_curr_ = odometry.pose.pose.position.x;
  y_curr_ = odometry.pose.pose.position.y;
  tf::Quaternion quat(odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w);
  quat = quat.normalize();
  theta_curr_ = set_angle_to_range(tf::getYaw(quat),0);

  ROS_DEBUG("Odometry set correctly");
  return true;
}

/// =======================================================================================
/// angularDiff (const geometry_msgs::Twist& heading, const geometry_msgs::Pose& pose)
/// =======================================================================================
double angularDiff (const geometry_msgs::Twist& heading,
    const geometry_msgs::Pose& pose)
{
  const double pi = 3.14159265;
  const double t1 = atan2(heading.linear.y, heading.linear.x);
  const double t2 = tf::getYaw(pose.orientation);
  const double d = t1-t2;

  if (fabs(d)<pi)
    return d;
  else if (d<0)
    return d+2*pi;
  else
    return d-2*pi;
}

/// ==================================================================================
/// Srl_local_planner::transformPose(geometry_msgs::PoseStamped init_pose)
/// Transforms the init_pose in the planner_frame
/// ==================================================================================
geometry_msgs::PoseStamped SrlEBandTrajectoryCtrl::transformPose(geometry_msgs::PoseStamped init_pose){

    geometry_msgs::PoseStamped res;
    tf::StampedTransform transform;
    try{
        // will transform data in the goal_frame into the planner_frame_
      //  tf_listener->waitForTransform( planner_frame_, init_pose.header.frame_id, ros::Time::now(), ros::Duration(0.40));
       tf_listener->lookupTransform(  planner_frame_, init_pose.header.frame_id, ros::Time(0), transform);

    }
    catch(tf::TransformException e){

        ROS_ERROR("Failed to transform the given pose in the Eband Planner frame_id, planner frame %s, inititpose frame %s, reason %s", planner_frame_.c_str(), init_pose.header.frame_id.c_str(), e.what());
    }

    tf::Pose source;
    tf::Quaternion q(init_pose.pose.orientation.x, init_pose.pose.orientation.y, init_pose.pose.orientation.z, init_pose.pose.orientation.w);
    q = q.normalize();

    // tf::Quaternion q = tf::createQuaternionFromRPY(0,0,tf::getYaw(init_pose.pose.orientation));
    tf::Matrix3x3 base(q);
    source.setOrigin(tf::Vector3(init_pose.pose.position.x, init_pose.pose.position.y, 0));
    source.setBasis(base);

    /// Apply the proper transform
    tf::Pose result = transform*source;
    res.pose.position.x = result.getOrigin().x() ;
    res.pose.position.y = result.getOrigin().y() ;
    res.pose.position.z = result.getOrigin().z() ;
    tf:: Quaternion qo = result.getRotation();
    qo.normalize();
    tf::quaternionTFToMsg(qo, res.pose.orientation);
    res.header = init_pose.header;
    res.header.frame_id = planner_frame_;

    return res;

}


/// ========================================================================================
/// compute_sigma(double vsig, double wsig), function to compute sigma, used as scaling
/// factor in the velocities computation when one of them is higher than the hardware limits
/// ========================================================================================
double SrlEBandTrajectoryCtrl::compute_sigma(double vsig, double wsig) {
    double max = vsig;

    if (wsig > max) {
      max = wsig;
    }

    if (1 > max) {
      max = 1;
    }

    return max;
}

/// =======================================================================================
/// set_angle_to_range(double alpha, double min), function used to map the angle beetween a
/// range of [min , min+2pi]
/// =======================================================================================
double SrlEBandTrajectoryCtrl::set_angle_to_range(double alpha, double min)
{

    while (alpha >= min + 2.0 * M_PI) {
        alpha -= 2.0 * M_PI;
    }
    while (alpha < min) {
        alpha += 2.0 * M_PI;
    }
    return alpha;
}



/// =======================================================================================
/// getTwistUnicycle(geometry_msgs::Twist& twist_cmd, bool& goal_reached)
/// =======================================================================================
bool SrlEBandTrajectoryCtrl::getTwistUnicycle(geometry_msgs::Twist& twist_cmd, bool& goal_reached){

  goal_reached = false;
  int size_band = elastic_band_.size();

  geometry_msgs::Twist robot_cmd, bubble_diff;
  robot_cmd.linear.x = 0.0;
  robot_cmd.angular.z = 0.0;

  bool command_provided = false;

  // check if plugin initialized
  if(!initialized_)	{
    ROS_ERROR("Requesting feedforward command from not initialized planner. Please call initialize() before using this planner");
    return false;
  }

  // check there is a plan at all (minimum 1 frame in this case, as robot + goal = plan)
  if( (!band_set_) || ( size_band < 2) ) {
    ROS_WARN("Requesting feedforward command from empty band.");
    return false;
  }

  // Get the differences between the first 2 bubbles in the robot's frame
  bubble_diff = getFrame1ToFrame2InRefFrameNew(
      elastic_band_.at(0).center.pose,
      elastic_band_.at(1).center.pose,
      elastic_band_.at(0).center.pose);


  float distance_from_goal = -1.0f;

  geometry_msgs::Twist bubble_diff_to_the_goal = getFrame1ToFrame2InRefFrameNew(
      elastic_band_.at(0).center.pose,
      elastic_band_.at(size_band-1).center.pose,
      elastic_band_.at(0).center.pose);


  //Check 0, Turn on the spot if the robot pose is flipped
  if (!command_provided && initial_turn_ && (fabs(bubble_diff_to_the_goal.linear.x) > 0.6 * tolerance_trans_ &&
      fabs(bubble_diff_to_the_goal.linear.y) > 0.6 * tolerance_trans_)  ) {

    ROS_WARN("Turning on the spot starting");
    // Storing second point of the band to turn to that direction
    if(initial_band_ == false){
        int index_for_direction = 1;

        if(elastic_band_.size()>3)
          index_for_direction=2;

        x_initial_band_ = elastic_band_.at(index_for_direction).center.pose.position.x;
        y_initial_band_ = elastic_band_.at(index_for_direction).center.pose.position.y;
        theta_initial_band_ =  angles::normalize_angle( tf::getYaw(elastic_band_.at(index_for_direction).center.pose.orientation) - tf::getYaw(elastic_band_.at(0).center.pose.orientation) );
        initial_band_ = true;
    }
    // Look for current robot pose
    tf::StampedTransform transform_flipped;
    try{
        tf_listener->lookupTransform("odom", robot_frame_, ros::Time(0), transform_flipped);
    }
    catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return false;
    }

    double x_rob = transform_flipped.getOrigin().x();
    double y_rob = transform_flipped.getOrigin().y();
    tf::Quaternion qcurr_flipped = transform_flipped.getRotation();
    qcurr_flipped.normalize();
    double robot_yaw = tf::getYaw(qcurr_flipped);
    double next_ball_yaw = theta_initial_band_;

    float next_orientation_diff = angles::normalize_angle(next_ball_yaw - robot_yaw);

    if (fabs(next_orientation_diff) > rot_stopping_turn_on_the_spot_) {

      initial_turn_ = true;
      ROS_WARN("Performing in place rotation for initial turning on the spot (robot_yaw, next_ball_yaw, diff): (%f, %f, %f)",robot_yaw, next_ball_yaw,  next_orientation_diff);
      double rotation_sign = -2 * (next_orientation_diff < 0) + 1;
      robot_cmd.angular.z =
        rotation_sign * min_in_place_vel_th_ + k_p_ * next_orientation_diff;
      if (fabs(robot_cmd.angular.z) > max_vel_th_) { // limit max rotation
        robot_cmd.angular.z = rotation_sign * max_vel_th_;
      }
    } else {
      initial_turn_ = false;
      ROS_WARN("Initial Turning on the spot completed");
    }
    command_provided = true;
  }

  // Check 1
  // We need to check if we are within the threshold of the final destination
  if (!command_provided) {
    int curr_target_bubble = 1;

    while(curr_target_bubble < (size_band) - 1) {
      curr_target_bubble++;
      bubble_diff =
        getFrame1ToFrame2InRefFrameNew(
            elastic_band_.at(0).center.pose,
            elastic_band_.at(curr_target_bubble).center.pose,
            elastic_band_.at(0).center.pose);
    }

    // if you go past tolerance, then try to get closer again
    if (!disallow_hysteresis_) {
      if(fabs(bubble_diff.linear.x) > tolerance_trans_ ||
          fabs(bubble_diff.linear.y) > tolerance_trans_) {
        in_final_goal_turn_ = false;
        ROS_DEBUG("if you go past tolerance, then try to get closer again, Toll Trans %f", tolerance_trans_ );
      }
    }

    // Get the differences between the first 2 bubbles in the robot's frame
    int goal_bubble = ((size_band) - 1);
    bubble_diff = getFrame1ToFrame2InRefFrameNew(
        elastic_band_.at(0).center.pose,
        elastic_band_.at(goal_bubble).center.pose,
        elastic_band_.at(0).center.pose);

    distance_from_goal = sqrtf(bubble_diff.linear.x * bubble_diff.linear.x +
                               bubble_diff.linear.y * bubble_diff.linear.y);

    // Get closer to the goal than the tolerance requires before starting the
    // final turn. The final turn may cause you to move slightly out of
    // position
    // tolerance_trans_
    if((fabs(bubble_diff.linear.x) <= 0.6 * b_ &&
        fabs(bubble_diff.linear.y) <= 0.6 * b_) ||
        in_final_goal_turn_) {
      // Calculate orientation difference to goal orientation (not captured in bubble_diff)
      double robot_yaw = tf::getYaw(elastic_band_.at(0).center.pose.orientation);
      double goal_yaw = tf::getYaw(elastic_band_.at((int)elastic_band_.size() - 1).center.pose.orientation);
      float orientation_diff = angles::normalize_angle(goal_yaw - robot_yaw);
      if (fabs(orientation_diff) > tolerance_rot_) {
        in_final_goal_turn_ = true;
        ROS_DEBUG("Performing in place rotation for goal (robot_yaw, goal_yaw, diff): (%f, %f, %f)",robot_yaw, goal_yaw, orientation_diff);
        double rotation_sign = -2 * (orientation_diff < 0) + 1;
        robot_cmd.angular.z =
          rotation_sign * min_in_place_vel_th_ + k_p_ * orientation_diff;
        if (fabs(robot_cmd.angular.z) > max_vel_th_) { // limit max rotation
          robot_cmd.angular.z = rotation_sign * max_vel_th_;
        }
      } else {
        in_final_goal_turn_ = false; // Goal reached
        ROS_INFO ("TrajectoryController: Goal reached with distance %.2f, %.2f (od = %.2f)"
            "; sending zero velocity",
            bubble_diff.linear.x, bubble_diff.linear.y, orientation_diff);
        // goal position reached
        robot_cmd.linear.x = 0.0;
        robot_cmd.angular.z = 0.0;
        goal_reached = true;
        initial_turn_ = true;
      }
      command_provided = true;
    }
  }

  // Check 2 - If we reach here, it means we need to use our PID controller to
  // move towards the next bubble
  if (!command_provided) {

    // Look for current robot pose
    ROS_DEBUG("Looking for current robot pose");
    tf::StampedTransform transform;
    try{
        tf_listener->lookupTransform("odom", "base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return false;
    }

    x_curr_ = transform.getOrigin().x();
    y_curr_ = transform.getOrigin().y();
    ROS_DEBUG("Looking for current robot pose, getting quaternion");
    tf::Quaternion qcurr = transform.getRotation();
    qcurr.normalize();
    ROS_DEBUG("Looking for current robot pose, getting yaw angle");
    theta_curr_ = set_angle_to_range(tf::getYaw(qcurr), 0);



    double err_x, vdx, feedback_v, feedback_w;
    double err_y, vdy;

    if(curr_control_index_> (int) plan_.size()){

        ROS_ERROR("Elastic band error not so many points into the planner");

    }

    int ind_band = lookahed_;
    if( size_band < 3 ){
      ind_band = 1;
    }else if(size_band < 2){
      ind_band = 0;
    }

    ROS_DEBUG("Before Transform pose");
    /// need to trasform path points in odom frame
    geometry_msgs::PoseStamped p;
    p.header = elastic_band_.at(ind_band).center.header;
    p.pose.position.x =  elastic_band_[ind_band].center.pose.position.x;
    p.pose.position.y =  elastic_band_[ind_band].center.pose.position.y;
    p.pose.orientation.x = elastic_band_[ind_band].center.pose.orientation.x;
    p.pose.orientation.y = elastic_band_[ind_band].center.pose.orientation.y;
    p.pose.orientation.z = elastic_band_[ind_band].center.pose.orientation.z;
    p.pose.orientation.w = elastic_band_[ind_band].center.pose.orientation.w;
    geometry_msgs::PoseStamped pt = transformPose(p);

    double vy_feed = 0;
    double vx_feed = 0;

    if(smoothed_eband_){

      err_y = ( plan_[ind_band].pose.position.y -(y_curr_+b_*sin(theta_curr_)));
      err_x = ( plan_[ind_band].pose.position.x -(x_curr_+b_*cos(theta_curr_)));

      vx_feed = dx_d_[ind_band];
      vy_feed = dy_d_[ind_band];

      vdx = vx_feed + k_one_*err_x ;
      vdy = vy_feed + k_two_*err_y ;

    } else{

      err_y = -1*( pt.pose.position.y -(y_curr_+b_*sin(theta_curr_)));
      err_x = -1*( pt.pose.position.x -(x_curr_+b_*cos(theta_curr_)));

      vx_feed = err_x*ts_;
      vy_feed = err_y*ts_;

      vdx = vx_feed + k_one_*err_x ;
      vdy = vy_feed + k_two_*err_y ;

    }


    ROS_DEBUG("Robot pose %f %f %f, control index %d ", x_curr_, y_curr_, theta_curr_, curr_control_index_);
    ROS_DEBUG("Next ball elastinc band %f %f", plan_[curr_control_index_].pose.position.x,
    plan_[curr_control_index_].pose.position.y );
    ROS_DEBUG("Errors in x %f and y %f", err_x, err_y);
    ROS_DEBUG("Feedforwad Vel in x %f and y %f", vx_feed, vy_feed);

    feedback_v = vdx*cos(theta_curr_)+vdy*sin(theta_curr_);
    feedback_w = (-1/b_)*vdx*sin(theta_curr_)+(1/b_)*vdy*cos(theta_curr_);

    double vsig,wsig,sig;


    vsig=fabs(feedback_v)/max_vel_lin_;
    wsig=fabs(feedback_w)/max_vel_th_;

    sig=compute_sigma(vsig,wsig);


    int sign_v;
    int sign_w;
    if(feedback_v>0)
        sign_v=1;
    else
        sign_v=-1;

    if(feedback_w>0)
        sign_w=1;
    else
        sign_w=-1;


    if(sig==1){
        feedback_v = feedback_v;
        feedback_w = feedback_w;
    }

    if(sig==vsig){
        feedback_v = feedback_v*max_vel_lin_;
        feedback_w = feedback_w/sig;
    }

    if(sig==wsig){
        feedback_v=feedback_v/sig;
        feedback_w=feedback_w*max_vel_th_;
    }

    // TODO: more checks on the acceleration!!!``
    // geometry_msgs::Twist feedback, feedback_limit_acc;
    // feedback.linear.x = feedback_v;
    // feedback.angular.z = feedback_w;
    //
    // feedback_limit_acc = checkAccelerationBounds(feedback);
    // feedback_v = feedback_limit_acc.linear.x;
    // feedback_w = feedback_limit_acc.angular.z;

    // Scale velocity if it is over the thrs.
    double forward_sign = -2 * (bubble_diff.linear.x < 0) + 1;
    double max_vel_lin = max_vel_lin_;
    if (distance_from_goal < 0.75f) {
      max_vel_lin = (max_vel_lin < 0.3) ? 0.15 : max_vel_lin / 2;
    }

    double linear_velocity = feedback_v;
    linear_velocity *= cos(bubble_diff.angular.z); //decrease while turning

    if (fabs(linear_velocity) > max_vel_lin_) {
        linear_velocity = forward_sign * max_vel_lin_;
    } else if (fabs(linear_velocity) < min_vel_lin_) {
        linear_velocity = forward_sign * min_vel_lin_;

    }else{
    }


  // Scale velocity if it is over the thrs.
  double rotation_sign = -2 * (bubble_diff.angular.z < 0) + 1;
  double angular_velocity = feedback_w;
  if (fabs(angular_velocity) > max_vel_th_) {
    angular_velocity = rotation_sign * max_vel_th_;
  } else if (fabs(angular_velocity) < min_vel_th_) {
    angular_velocity = rotation_sign * min_vel_th_;
  }
    // robot_cmd.linear.x = feedback_v;
    // robot_cmd.angular.z = feedback_w;

    robot_cmd.linear.x = linear_velocity;
    robot_cmd.angular.z = angular_velocity;


    command_provided = true;
    curr_control_index_++;

  }

  twist_cmd = robot_cmd;


  ROS_DEBUG("Final command: %f, %f", twist_cmd.linear.x, twist_cmd.angular.z);
  return true;



}


/// =======================================================================================
/// checkAccelerationBounds(geometry_msgs::Twist& twist_cmd, bool& goal_reached)
/// =======================================================================================
geometry_msgs::Twist SrlEBandTrajectoryCtrl::checkAccelerationBounds(geometry_msgs::Twist twist_cmd){

  ROS_DEBUG("Initial Velocity, before checking acceleration bounds  %f %f", twist_cmd.linear.x, twist_cmd.angular.z);
  ROS_DEBUG("Last Velocity, before checking acceleration bounds  %f %f", last_vel_.linear.x, last_vel_.angular.z);
  ROS_DEBUG("Virtual Mass %f, Gain %f", virt_mass_, k_nu_);

  // CheckBounds on Accelerations
  geometry_msgs::Twist acc_desired;
  acc_desired = twist_cmd;
  acc_desired.linear.x = (1.0/virt_mass_) * k_nu_ * (twist_cmd.linear.x - last_vel_.linear.x);
  acc_desired.angular.z = (1.0/virt_mass_) * k_nu_ * (twist_cmd.angular.z - last_vel_.angular.z);
  ROS_DEBUG("accelerations  %f %f", acc_desired.linear.x, acc_desired.angular.z);


  // acc_desired.linear.x =  controller_frequency_*(twist_cmd.linear.x - last_vel_.linear.x);
  // acc_desired.angular.z = controller_frequency_* (twist_cmd.angular.z - last_vel_.angular.z);

  // constrain acceleration
  double scale_acc;
  double abs_acc_trans = sqrt( (acc_desired.linear.x*acc_desired.linear.x));
  if(abs_acc_trans > acc_max_trans_)
  {
    scale_acc = acc_max_trans_ / abs_acc_trans;
    acc_desired.linear.x *= scale_acc;
    acc_desired.angular.z *= scale_acc;
  }

  if(fabs(acc_desired.angular.z) > acc_max_rot_)
  {
    scale_acc = fabs(acc_desired.angular.z) / acc_max_rot_;
    acc_desired.angular.z *= scale_acc;
    acc_desired.linear.x *= scale_acc;

  }

  ROS_DEBUG("Scaled accelerations  %f %f", acc_desired.linear.x, acc_desired.angular.z);
  // and get velocity-cmds by integrating them over one time-step
  last_vel_.linear.x = last_vel_.linear.x + acc_desired.linear.x / controller_frequency_;
  last_vel_.angular.z = last_vel_.angular.z + acc_desired.angular.z / controller_frequency_;
  ROS_DEBUG("Last Velocity %f %f", last_vel_.linear.x, last_vel_.angular.z);
  ROS_DEBUG("Last Velocity Limited %f %f", last_vel_.linear.x, last_vel_.angular.z);

  // finally set robot_cmd (to non-zero value)
  return last_vel_;


}
/// =======================================================================================
/// limitVelocityDensityLaserPoints(), define upper bound velocity according to density of laser points, and band direction
/// =======================================================================================
bool SrlEBandTrajectoryCtrl::limitVelocityDensityLaserPoints(double &curr_max_vel, double band_dir){

    /// Moving backward rear_laser counts
    if(fabs(band_dir)<=M_PI/2 ){
        if(num_points_front_robot_ > 75){
            curr_max_vel = max_translational_vel_due_to_laser_points_density_;
            ROS_DEBUG("Limiting Velocity due to too many Laser scans");
        }
    }
    else /// Moving forward front_laser counts
    {
      if(num_points_rear_robot_ > 75){
          curr_max_vel = max_translational_vel_due_to_laser_points_density_;
          ROS_DEBUG("Limiting Velocity due to too many Laser scans in front of the robot (Rear)");
      }

    }

    return true;
}

bool SrlEBandTrajectoryCtrl::limitVelocityHRI(double &curr_max_vel){

  if(curr_max_vel>max_vel_th_hri_)
    curr_max_vel = max_vel_th_hri_;

    return true;


}
/// =======================================================================================
/// limitVelocityCurvature()
/// =======================================================================================
bool SrlEBandTrajectoryCtrl::limitVelocityCurvature(double &curr_max_vel){


      int size_elastic_band = elastic_band_.size();
      if( (!band_set_) || ( size_elastic_band < 2) ) {
        ROS_WARN("Requesting computation max curvature from empty band.");
        return false;
      }

      vector<double> x_bubbles;
      vector<double> y_bubbles;

      ROS_DEBUG("Copying path from elastic band of length %d", size_elastic_band);

      for(int i=0; i<size_elastic_band; i++){
          geometry_msgs::Pose bubble_pose = elastic_band_.at(i).center.pose;
          x_bubbles.push_back(bubble_pose.position.x);
          y_bubbles.push_back(bubble_pose.position.y);
      }

      double maxK = compute_curvature_properties_->pathMaxCurvature(x_bubbles, y_bubbles);
      int index_maxK = compute_curvature_properties_->getLastIndexMaxCurvature();
      ROS_DEBUG("Max Curvature %f, index %d", maxK, index_maxK);
      ROS_DEBUG("Current Max Velocity %f", curr_max_vel);
      double ds_i = curr_max_vel* (1/controller_frequency_);
      /// get distance to the point with max curvature
      if(index_maxK>=size_elastic_band)
        return false;

      geometry_msgs::Twist diff = getFrame1ToFrame2InRefFrameNew(
           elastic_band_.at(0).center.pose,
           elastic_band_.at(index_maxK).center.pose,
           elastic_band_.at(0).center.pose);

      double ds_maxK =  sqrtf(diff.linear.x * diff.linear.x +
                                     diff.linear.y * diff.linear.y);

      /// Reduce VELOCITY_COMMAND if Curvature is high
      // and according to the space of reaction
      if(maxK >= curvature_guarding_thrs_){
        ROS_DEBUG("ds_i %f, ds_maxK %f", ds_i, ds_maxK);

        if(ds_maxK>2*ds_i && ds_maxK<4*ds_i)
        {
          curr_max_vel = 0.75*curr_max_vel;

        }else if(ds_maxK > ds_i && ds_maxK<2*ds_i){

          curr_max_vel = 0.50*curr_max_vel;

        }else if(ds_maxK < ds_i ){
          curr_max_vel = 0.35*curr_max_vel;

        }
      }

      if(curr_max_vel<min_vel_limited_curvature_)
        curr_max_vel = min_vel_limited_curvature_;

      ROS_DEBUG("Final Max Velocity after limiting the curvature %f", curr_max_vel);

      return true;
}



/// =======================================================================================
/// getTwistDifferentialDrive(geometry_msgs::Twist& twist_cmd, bool& goal_reached)
/// =======================================================================================
bool SrlEBandTrajectoryCtrl::getTwistDifferentialDrive(geometry_msgs::Twist& twist_cmd, bool& goal_reached) {
  goal_reached = false;

  geometry_msgs::Twist robot_cmd, bubble_diff;
  robot_cmd.linear.x = 0.0;
  robot_cmd.angular.z = 0.0;
  ROS_DEBUG("Human Legibility on");
  // get current robot pose
  tf::StampedTransform transform_flipped;
  try{
      tf_listener->lookupTransform("odom", robot_frame_ , ros::Time(0), transform_flipped);
  }
  catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          return false;
  }

  double x_rob = transform_flipped.getOrigin().x();
  double y_rob = transform_flipped.getOrigin().y();
  tf::Quaternion qcurr_flipped = transform_flipped.getRotation();
  qcurr_flipped.normalize();
  double robot_yaw = tf::getYaw(qcurr_flipped);
  bool command_provided = false;

  // check if plugin initialized
  if(!initialized_)	{
    ROS_ERROR("Requesting feedforward command from not initialized planner. Please call initialize() before using this planner");
    return false;
  }

  // check there is a plan at all (minimum 1 frame in this case, as robot + goal = plan)
  if( (!band_set_) || (elastic_band_.size() < 2) ) {
    ROS_WARN("Requesting feedforward command from empty band.");
    return false;
  }

  // Get the differences between the first 2 bubbles in the robot's frame
  bubble_diff = getFrame1ToFrame2InRefFrameNew(
      elastic_band_.at(0).center.pose,
      elastic_band_.at(1).center.pose,
      elastic_band_.at(0).center.pose);

  float distance_from_goal = -1.0f;

  // Check 1
  // We need to check if we are within the threshold of the final destination
  if (!command_provided) {
    int curr_target_bubble = 1;

    while(curr_target_bubble < ((int) elastic_band_.size()) - 1) {
      curr_target_bubble++;
      bubble_diff =
        getFrame1ToFrame2InRefFrameNew(
            elastic_band_.at(0).center.pose,
            elastic_band_.at(curr_target_bubble).center.pose,
            elastic_band_.at(0).center.pose);
    }

    // if you go past tolerance, then try to get closer again
    if (!disallow_hysteresis_) {
      if(fabs(bubble_diff.linear.x) > tolerance_trans_ ||
          fabs(bubble_diff.linear.y) > tolerance_trans_) {
        in_final_goal_turn_ = false;
      }
    }

    // Get the differences between the first 2 bubbles in the robot's frame
    int goal_bubble = (((int) elastic_band_.size()) - 1);
    bubble_diff = getFrame1ToFrame2InRefFrameNew(
        elastic_band_.at(0).center.pose,
        elastic_band_.at(goal_bubble).center.pose,
        elastic_band_.at(0).center.pose);

    distance_from_goal = sqrtf(bubble_diff.linear.x * bubble_diff.linear.x +
                               bubble_diff.linear.y * bubble_diff.linear.y);

    // Get closer to the goal than the tolerance requires before starting the
    // final turn. The final turn may cause you to move slightly out of
    // position
    if((fabs(bubble_diff.linear.x) <= 0.6 * tolerance_trans_ &&
        fabs(bubble_diff.linear.y) <= 0.6 * tolerance_trans_ ) ||
        (in_final_goal_turn_ && elastic_band_.size() < 3) ) {
      // Calculate orientation difference to goal orientation (not captured in bubble_diff)
       double robot_yaw_to_goal = tf::getYaw(elastic_band_.at(0).center.pose.orientation);
      //double robot_yaw_to_goal = robot_yaw;
      double goal_yaw = tf::getYaw(elastic_band_.at((int)elastic_band_.size() - 1).center.pose.orientation);
      float orientation_diff = angles::normalize_angle(goal_yaw - robot_yaw_to_goal);
      if (fabs(orientation_diff) > tolerance_rot_) {
        in_final_goal_turn_ = true;
        double rotation_sign = -2 * (orientation_diff < 0) + 1;
        robot_cmd.angular.z = rotation_sign * min_in_place_vel_th_ + k_p_ * orientation_diff;

        // if (fabs(robot_cmd.angular.z) > max_vel_th_) { // limit max rotation
        //   robot_cmd.angular.z = rotation_sign * max_vel_th_;
        // }

        if (fabs(robot_cmd.angular.z) > max_rotational_velocity_turning_on_spot_) { // limit max rotation
          robot_cmd.angular.z = rotation_sign * max_rotational_velocity_turning_on_spot_;
        }
          ROS_DEBUG("Performing in place rotation for goal (diff,w): %f %f", orientation_diff, robot_cmd.angular.z );


      } else {
            in_final_goal_turn_ = false; // Goal reached
            ROS_INFO ("TrajectoryController: Goal reached with distance %.2f, %.2f (od = %.2f)"
            "; sending zero velocity",
            bubble_diff.linear.x, bubble_diff.linear.y, orientation_diff);
            // goal position reached
            robot_cmd.linear.x = 0.0;
            robot_cmd.angular.z = 0.0;
            goal_reached = true;
            return true;
      }
      command_provided = true;
    }
  }

  // Get the differences between the first 2 bubbles in the robot's frame
  bubble_diff = getFrame1ToFrame2InRefFrameNew(
      elastic_band_.at(0).center.pose,
      elastic_band_.at(1).center.pose,
      elastic_band_.at(0).center.pose);

  // Check 1 - check if the robot's current pose is too misaligned with the next bubble
  if (!command_provided) {
    ROS_DEBUG("Goal has not been reached, performing checks to move towards goal");

    // calculate an estimate of the in-place rotation threshold
    double distance_to_next_bubble = sqrt(
        bubble_diff.linear.x * bubble_diff.linear.x +
        bubble_diff.linear.y * bubble_diff.linear.y);
    double radius_of_next_bubble = 0.7 * elastic_band_.at(1).expansion;
    double in_place_rotation_threshold =
      rotation_threshold_multiplier_ *
      fabs(atan2(radius_of_next_bubble,distance_to_next_bubble));
    ROS_DEBUG("In-place rotation threshold: %f(%f,%f)",
        in_place_rotation_threshold, radius_of_next_bubble, distance_to_next_bubble);

    // check if we are above this threshold, if so then perform in-place rotation
    if (fabs(bubble_diff.angular.z) > in_place_rotation_threshold) {
      robot_cmd.angular.z = k_p_ * bubble_diff.angular.z;
      double rotation_sign = (bubble_diff.angular.z < 0) ? -1.0 : +1.0;
      if (fabs(robot_cmd.angular.z) < min_in_place_vel_th_) {
        robot_cmd.angular.z = rotation_sign * min_in_place_vel_th_;
      }
     //  if (fabs(robot_cmd.angular.z) > max_vel_th_) { // limit max rotation
       //  robot_cmd.angular.z = rotation_sign * max_vel_th_;
      // }
      if (fabs(robot_cmd.angular.z) > max_rotational_velocity_turning_on_spot_) { // limit max rotation
           robot_cmd.angular.z = rotation_sign * max_rotational_velocity_turning_on_spot_;
        }

        ROS_DEBUG("Performing in place rotation for start (diff): %f", bubble_diff.angular.z, robot_cmd.angular.z);
      command_provided = true;
    }
  }

  // Check 3 - If we reach here, it means we need to use our PID controller to
  // move towards the next bubble
  if (!command_provided) {

    // Select a linear velocity (based on the current bubble radius)
    double forward_sign = -2 * (bubble_diff.linear.x < 0) + 1;
    double bubble_radius = 0.7 * elastic_band_.at(0).expansion;
    double velocity_multiplier = bubble_velocity_multiplier_ * bubble_radius;

    double max_vel_lin = max_vel_lin_;
    if (distance_from_goal < start_to_stop_goal_) {
      //max_vel_lin = (max_vel_lin < 0.3) ? 0.15 : max_vel_lin / 2 ;
      max_vel_lin = (max_vel_lin < 0.3) ? 0.15 : trans_vel_goal_ ;
    }

    ROS_DEBUG("Bubble gain %f, bubble_radius %f, velocity_multiplier %f, max_vel_lin %f, max_vel_lin_ %f, distance to the goal %f", bubble_velocity_multiplier_, bubble_radius,
                velocity_multiplier, max_vel_lin, max_vel_lin_, distance_from_goal);

    double linear_velocity = velocity_multiplier * max_vel_lin;
    ROS_DEBUG("Linar Velocity before checking the turn %f", linear_velocity);
    linear_velocity *= cos(2*bubble_diff.angular.z); //decrease while turning

    /// Verify now limits

    if(limit_vel_based_laser_points_density_)
      limitVelocityDensityLaserPoints(linear_velocity, bubble_diff.angular.z);

    if(limit_vel_based_on_curvature_)
      limitVelocityCurvature(linear_velocity);

    if(limit_vel_based_on_hri_)
      limitVelocityHRI(linear_velocity);

    ROS_DEBUG("Linar Velocity before after the turn %f", linear_velocity);
    if (fabs(linear_velocity) > max_vel_lin_) {
      linear_velocity = forward_sign * max_vel_lin_;
    } else if (fabs(linear_velocity) < min_vel_lin_) {
      linear_velocity = forward_sign * min_vel_lin_;
    }

    // Select an angular velocity (based on PID controller)
    double error = bubble_diff.angular.z;
    integral_angular_ = integral_angular_ + error*(1/controller_frequency_);
    double derivative_angular = (error - previous_angular_error_)*controller_frequency_;

    double rotation_sign = -2 * (bubble_diff.angular.z < 0) + 1;
    /// Only Propotinal term
    double angular_velocity = k_p_ * error;

    // double angular_velocity = k_p_ * error + k_one_ * integral_angular_ + k_two_*derivative_angular;

    if (fabs(angular_velocity) > max_vel_th_) {
      angular_velocity = rotation_sign * max_vel_th_;
    } else if (fabs(angular_velocity) < min_vel_th_) {
      angular_velocity = rotation_sign * min_vel_th_;
    }



    if(human_legibility_on_){

      // generate scaling factores
      base_local_planner::Trajectory local_path;
      context_cost_function_->generateTrajectory(x_rob, y_rob, robot_yaw,
          linear_velocity, angular_velocity, local_path, backward_motion_on_, false);
      double trajectory_scale = context_cost_function_->scoreTrajectory(local_path);



      if(trajectory_scale<1.0){

        linear_velocity =  trajectory_scale * linear_velocity;
        angular_velocity = trajectory_scale * angular_velocity;
        /// generate and publish new local trajectory which implements the human awareness
        base_local_planner::Trajectory scaled_local_path;
        context_cost_function_->generateTrajectory(x_rob, y_rob, robot_yaw,
            linear_velocity, angular_velocity, scaled_local_path, backward_motion_on_, true);

        publishLocalPlan(scaled_local_path);

      }else{

        publishLocalPlan(local_path);
      }

    }


    ROS_DEBUG("Selected velocity: lin: %f, ang: %f",
        linear_velocity, angular_velocity);

    robot_cmd.linear.x = linear_velocity;
    robot_cmd.angular.z = angular_velocity;

    previous_angular_error_ = error;
    command_provided = true;
  }

  twist_cmd = robot_cmd;

  ROS_DEBUG("Final command: %f, %f", twist_cmd.linear.x, twist_cmd.angular.z);
  return true;
}



bool SrlEBandTrajectoryCtrl::getTwist(geometry_msgs::Twist& twist_cmd, bool& goal_reached)
{
  goal_reached = false;
  if (differential_drive_on_) {
    bool res = false;
    if(!tracker_on_)
      res = getTwistDifferentialDrive(twist_cmd, goal_reached);
    else
      res = getTwistUnicycle(twist_cmd, goal_reached);

      if(isfinite(twist_cmd.linear.x) && isfinite(twist_cmd.angular.z))
          return res;
        else
        return false;

  }

  // init twist cmd to be handed back to caller
  geometry_msgs::Twist robot_cmd, bubble_diff, control_deviation;
  robot_cmd.linear.x = 0.0;
  robot_cmd.linear.y = 0.0;
  robot_cmd.linear.z = 0.0;
  robot_cmd.angular.x = 0.0;
  robot_cmd.angular.y = 0.0;
  robot_cmd.angular.z = 0.0;

  // make sure command vector is clean
  twist_cmd = robot_cmd;
  control_deviation = robot_cmd;

  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("Requesting feedforward command from not initialized planner. Please call initialize() before using this planner");
    return false;
  }

  // check there is a plan at all (minimum 1 frame in this case, as robot + goal = plan)
  if( (!band_set_) || (elastic_band_.size() < 2) )
  {
    ROS_WARN("Requesting feedforward command from empty band.");
    return false;
  }

  // calc intersection of bubble-radius with sequence of vector connecting the bubbles

  // get distance to target from bubble-expansion
  double scaled_radius = 0.7 * elastic_band_.at(0).expansion;

  // get difference and distance between bubbles in odometry frame
  double bubble_distance, ang_pseudo_dist;
  bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(0).center.pose,
      elastic_band_.at(1).center.pose,
      ref_frame_band_);
  ang_pseudo_dist = bubble_diff.angular.z * circumscribed_radius_;
  bubble_distance = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) + (bubble_diff.linear.y * bubble_diff.linear.y) +
      (ang_pseudo_dist * ang_pseudo_dist) );

  if(visualization_)
  {
    target_visual_->publishBubble("ctrl_target", 1, target_visual_->blue, elastic_band_.at(0));
    target_visual_->publishBubble("ctrl_target", 2, target_visual_->blue, elastic_band_.at(1));
  }

  // by default our control deviation is the difference between the bubble centers
  double abs_ctrl_dev;
  control_deviation = bubble_diff;


  ang_pseudo_dist = control_deviation.angular.z * circumscribed_radius_;
  abs_ctrl_dev = sqrt( (control_deviation.linear.x * control_deviation.linear.x) +
      (control_deviation.linear.y * control_deviation.linear.y) +
      (ang_pseudo_dist * ang_pseudo_dist) );

  // yet depending on the expansion of our bubble we might want to adapt this point
  if(scaled_radius < bubble_distance)
  {
    // triviale case - simply scale bubble_diff
    double scale_difference = scaled_radius / bubble_distance;
    bubble_diff.linear.x *= scale_difference;
    bubble_diff.linear.y *= scale_difference;
    bubble_diff.angular.z *= scale_difference;
    // set controls
    control_deviation = bubble_diff;
  }

  // if scaled_radius = bubble_distance -- we have nothing to do at all

  if(scaled_radius > bubble_distance)
  {
    // o.k. now we have to do a little bit more -> check next but one bubble
    if(elastic_band_.size() > 2)
    {
      // get difference between next and next but one bubble
      double next_bubble_distance;
      geometry_msgs::Twist next_bubble_diff;
      next_bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(1).center.pose,
          elastic_band_.at(2).center.pose,
          ref_frame_band_);
      ang_pseudo_dist = next_bubble_diff.angular.z * circumscribed_radius_;
      next_bubble_distance = sqrt( (next_bubble_diff.linear.x * next_bubble_diff.linear.x) +
          (next_bubble_diff.linear.y * next_bubble_diff.linear.y) +
          (ang_pseudo_dist * ang_pseudo_dist) );

      if(scaled_radius > (bubble_distance + next_bubble_distance) )
      {
        // we should normally not end up here - but just to be sure
        control_deviation.linear.x = bubble_diff.linear.x + next_bubble_diff.linear.x;
        control_deviation.linear.y = bubble_diff.linear.y + next_bubble_diff.linear.y;
        control_deviation.angular.z = bubble_diff.angular.z + next_bubble_diff.angular.z;
        // done
        if(visualization_)
          target_visual_->publishBubble("ctrl_target", 3, target_visual_->red, elastic_band_.at(2));
      }
      else
      {
        if(visualization_)
          target_visual_->publishBubble("ctrl_target", 3, target_visual_->red, elastic_band_.at(2));

        // we want to calculate intersection point of bubble ...
        // ... and vector connecting the following bubbles
        double b_distance, cosine_at_bub;
        double vec_prod, norm_vec1, norm_vec2;
        double ang_pseudo_dist1, ang_pseudo_dist2;

        // get distance between next bubble center and intersection point
        ang_pseudo_dist1 = bubble_diff.angular.z * circumscribed_radius_;
        ang_pseudo_dist2 = next_bubble_diff.angular.z * circumscribed_radius_;
        // careful! - we need this sign because of the direction of the vectors and the definition of the vector-product
        vec_prod = - ( (bubble_diff.linear.x * next_bubble_diff.linear.x) +
            (bubble_diff.linear.y * next_bubble_diff.linear.y) +
            (ang_pseudo_dist1 * ang_pseudo_dist2) );

        norm_vec1 = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) +
            (bubble_diff.linear.y * bubble_diff.linear.y) +
            (ang_pseudo_dist1 * ang_pseudo_dist1) );

        norm_vec2 = sqrt( (next_bubble_diff.linear.x * next_bubble_diff.linear.x) +
            (next_bubble_diff.linear.y * next_bubble_diff.linear.y) +
            (ang_pseudo_dist2 * ang_pseudo_dist2) );

        // reform the cosine-rule
        cosine_at_bub = vec_prod / norm_vec1 / norm_vec2;
        b_distance = bubble_distance * cosine_at_bub + sqrt( scaled_radius*scaled_radius -
            bubble_distance*bubble_distance * (1.0 - cosine_at_bub*cosine_at_bub) );

        // get difference vector from next_bubble to intersection point
        double scale_next_difference = b_distance / next_bubble_distance;
        next_bubble_diff.linear.x *= scale_next_difference;
        next_bubble_diff.linear.y *= scale_next_difference;
        next_bubble_diff.angular.z *= scale_next_difference;

        // and finally get the control deviation
        control_deviation.linear.x = bubble_diff.linear.x + next_bubble_diff.linear.x;
        control_deviation.linear.y = bubble_diff.linear.y + next_bubble_diff.linear.y;
        control_deviation.angular.z = bubble_diff.angular.z + next_bubble_diff.angular.z;
        // done
      }
    }
  }

  // plot control deviation
  ang_pseudo_dist = control_deviation.angular.z * circumscribed_radius_;
  abs_ctrl_dev = sqrt( (control_deviation.linear.x * control_deviation.linear.x) +
      (control_deviation.linear.y * control_deviation.linear.y) +
      (ang_pseudo_dist * ang_pseudo_dist) );


  if(visualization_)
  {
    // compose bubble from ctrl-target
    geometry_msgs::Pose2D tmp_bubble_2d, curr_bubble_2d;
    geometry_msgs::Pose tmp_pose;
    // init bubble for visualization
    Bubble new_bubble = elastic_band_.at(0);
    PoseToPose2D(elastic_band_.at(0).center.pose, curr_bubble_2d);
    tmp_bubble_2d.x = curr_bubble_2d.x + control_deviation.linear.x;
    tmp_bubble_2d.y = curr_bubble_2d.y + control_deviation.linear.y;
    tmp_bubble_2d.theta = curr_bubble_2d.theta + control_deviation.angular.z;
    Pose2DToPose(tmp_pose, tmp_bubble_2d);
    new_bubble.center.pose = tmp_pose;
    new_bubble.expansion = 0.1; // just draw a small bubble
    target_visual_->publishBubble("ctrl_target", 0, target_visual_->red, new_bubble);
  }


  const geometry_msgs::Point& goal = (--elastic_band_.end())->center.pose.position;
  const double dx = elastic_band_.at(0).center.pose.position.x - goal.x;
  const double dy = elastic_band_.at(0).center.pose.position.y - goal.y;
  const double dist_to_goal = sqrt(dx*dx + dy*dy);

  // Assuming we're far enough from the final goal, make sure to rotate so
  // we're facing the right way
  if (dist_to_goal > rotation_correction_threshold_)
  {

    const double angular_diff = angularDiff(control_deviation, elastic_band_.at(0).center.pose);
    const double vel = pid_.computeCommand(angular_diff, ros::Duration(1/controller_frequency_));
    const double mult = fabs(vel) > max_vel_th_ ? max_vel_th_/fabs(vel) : 1.0;
    control_deviation.angular.z = vel*mult;
    const double abs_vel = fabs(control_deviation.angular.z);

    ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
        "Angular diff is %.2f and desired angular "
        "vel is %.2f.  Initial translation velocity "
        "is %.2f, %.2f", angular_diff,
        control_deviation.angular.z,
        control_deviation.linear.x,
        control_deviation.linear.y);
    const double trans_mult = max(0.01, 1.0 - abs_vel/max_vel_th_); // There are some weird tf errors if I let it be 0
    control_deviation.linear.x *= trans_mult;
    control_deviation.linear.y *= trans_mult;
    ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
        "Translation multiplier is %.2f and scaled "
        "translational velocity is %.2f, %.2f",
        trans_mult, control_deviation.linear.x,
        control_deviation.linear.y);
  }
  else
    ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
        "Not applying angle correction because "
        "distance to goal is %.2f", dist_to_goal);




  // now the actual control procedure start (using attractive Potentials)
  geometry_msgs::Twist desired_velocity, currbub_maxvel_dir;
  double desvel_abs, desvel_abs_trans, currbub_maxvel_abs;
  double scale_des_vel;
  desired_velocity = robot_cmd;
  currbub_maxvel_dir = robot_cmd;

  // calculate "equilibrium velocity" (Khatib86 - Realtime Obstacle Avoidance)
  desired_velocity.linear.x = k_p_/k_nu_ * control_deviation.linear.x;
  desired_velocity.linear.y = k_p_/k_nu_ * control_deviation.linear.y;
  desired_velocity.angular.z = k_p_/k_nu_ * control_deviation.angular.z;

  //robot_cmd = desired_velocity;

  // get max vel for current bubble
  int curr_bub_num = 0;
  currbub_maxvel_abs = getBubbleTargetVel(curr_bub_num, elastic_band_, currbub_maxvel_dir);

  // if neccessarry scale desired vel to stay lower than currbub_maxvel_abs
  ang_pseudo_dist = desired_velocity.angular.z * circumscribed_radius_;
  desvel_abs = sqrt( (desired_velocity.linear.x * desired_velocity.linear.x) +
      (desired_velocity.linear.y * desired_velocity.linear.y) +
      (ang_pseudo_dist * ang_pseudo_dist) );
  if(desvel_abs > currbub_maxvel_abs)
  {
    scale_des_vel = currbub_maxvel_abs / desvel_abs;
    desired_velocity.linear.x *= scale_des_vel;
    desired_velocity.linear.y *= scale_des_vel;
    desired_velocity.angular.z *= scale_des_vel;
  }

  // make sure to stay within velocity bounds for the robot
  desvel_abs_trans = sqrt( (desired_velocity.linear.x * desired_velocity.linear.x) + (desired_velocity.linear.y * desired_velocity.linear.y) );
  // for translation
  if(desvel_abs_trans > max_vel_lin_)
  {
    scale_des_vel = max_vel_lin_ / desvel_abs_trans;
    desired_velocity.linear.x *= scale_des_vel;
    desired_velocity.linear.y *= scale_des_vel;
    // to make sure we are staying inside the bubble also scale rotation
    desired_velocity.angular.z *= scale_des_vel;
  }

  // for rotation
  if(fabs(desired_velocity.angular.z) > max_vel_th_)
  {
    scale_des_vel = max_vel_th_ / fabs(desired_velocity.angular.z);
    desired_velocity.angular.z *= scale_des_vel;
    // to make sure we are staying inside the bubble also scale translation
    desired_velocity.linear.x *= scale_des_vel;
    desired_velocity.linear.y *= scale_des_vel;
  }

  // calculate resulting force (accel. resp.) (Khatib86 - Realtime Obstacle Avoidance)
  geometry_msgs::Twist acc_desired;
  acc_desired = robot_cmd;
  acc_desired.linear.x = (1.0/virt_mass_) * k_nu_ * (desired_velocity.linear.x - last_vel_.linear.x);
  acc_desired.linear.y = (1.0/virt_mass_) * k_nu_ * (desired_velocity.linear.y - last_vel_.linear.y);
  acc_desired.angular.z = (1.0/virt_mass_) * k_nu_ * (desired_velocity.angular.z - last_vel_.angular.z);

  // constrain acceleration
  double scale_acc;
  double abs_acc_trans = sqrt( (acc_desired.linear.x*acc_desired.linear.x) + (acc_desired.linear.y*acc_desired.linear.y) );
  if(abs_acc_trans > acc_max_trans_)
  {
    scale_acc = acc_max_trans_ / abs_acc_trans;
    acc_desired.linear.x *= scale_acc;
    acc_desired.linear.y *= scale_acc;
    // again - keep relations - stay in bubble
    acc_desired.angular.z *= scale_acc;
  }

  if(fabs(acc_desired.angular.z) > acc_max_rot_)
  {
    scale_acc = fabs(acc_desired.angular.z) / acc_max_rot_;
    acc_desired.angular.z *= scale_acc;
    // again - keep relations - stay in bubble
    acc_desired.linear.x *= scale_acc;
    acc_desired.linear.y *= scale_acc;
  }

  // and get velocity-cmds by integrating them over one time-step
  last_vel_.linear.x = last_vel_.linear.x + acc_desired.linear.x / controller_frequency_;
  last_vel_.linear.y = last_vel_.linear.y + acc_desired.linear.y / controller_frequency_;
  last_vel_.angular.z = last_vel_.angular.z + acc_desired.angular.z / controller_frequency_;


  // we are almost done now take into accoun stick-slip and similar nasty things

  // last checks - limit current twist cmd (upper and lower bounds)
  last_vel_ = limitTwist(last_vel_);

  // finally set robot_cmd (to non-zero value)
  robot_cmd = last_vel_;

  // now convert into robot-body frame
  robot_cmd = transformTwistFromFrame1ToFrame2(robot_cmd, ref_frame_band_, elastic_band_.at(0).center.pose);

  // check whether we reached the end of the band
  int curr_target_bubble = 1;
  while(fabs(bubble_diff.linear.x) <= tolerance_trans_ &&
      fabs(bubble_diff.linear.y) <= tolerance_trans_ &&
      fabs(bubble_diff.angular.z) <= tolerance_rot_)
  {
    if(curr_target_bubble < ((int) elastic_band_.size()) - 1)
    {
      curr_target_bubble++;
      // transform next target bubble into robot-body frame
      // and get difference to robot bubble
      bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(0).center.pose, elastic_band_.at(curr_target_bubble).center.pose,
          ref_frame_band_);
    }
    else
    {
      ROS_DEBUG_THROTTLE_NAMED (1.0, "controller_state",
          "Goal reached with distance %.2f, %.2f, %.2f"
          "; sending zero velocity",
          bubble_diff.linear.x, bubble_diff.linear.y,
          bubble_diff.angular.z);
      // goal position reached
      robot_cmd.linear.x = 0.0;
      robot_cmd.linear.y = 0.0;
      robot_cmd.angular.z = 0.0;
      // reset velocity
      last_vel_.linear.x = 0.0;
      last_vel_.linear.y = 0.0;
      last_vel_.angular.z = 0.0;
      goal_reached = true;
      break;
    }
  }

  twist_cmd = robot_cmd;

  return true;
}


double SrlEBandTrajectoryCtrl::getBubbleTargetVel(const int& target_bub_num, const std::vector<Bubble>& band, geometry_msgs::Twist& VelDir)
{
  // init reference for direction vector
  VelDir.linear.x = 0.0;
  VelDir.linear.y = 0.0;
  VelDir.linear.z = 0.0;
  VelDir.angular.x = 0.0;
  VelDir.angular.y = 0.0;
  VelDir.angular.z = 0.0;

  // if we are looking at the last bubble - target vel is always zero
  if(target_bub_num >= ((int) band.size() - 1))
    return 0.0;


  // otherwise check for max_vel calculated from current bubble size
  double v_max_curr_bub, v_max_next_bub;
  double bubble_distance, angle_to_pseudo_vel, delta_vel_max;
  geometry_msgs::Twist bubble_diff;

  // distance for braking s = 0.5*v*v/a
  v_max_curr_bub = sqrt(2 * elastic_band_.at(target_bub_num).expansion * acc_max_);

  // get distance to next bubble center
  ROS_ASSERT( (target_bub_num >= 0) && ((target_bub_num +1) < (int) band.size()) );
  bubble_diff = getFrame1ToFrame2InRefFrame(band.at(target_bub_num).center.pose, band.at(target_bub_num + 1).center.pose,
      ref_frame_band_);
  angle_to_pseudo_vel = bubble_diff.angular.z * circumscribed_radius_;

  bubble_distance = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) + (bubble_diff.linear.y * bubble_diff.linear.y) +
      (angle_to_pseudo_vel * angle_to_pseudo_vel) );

  // calculate direction vector - norm of diference
  VelDir.linear.x =bubble_diff.linear.x/bubble_distance;
  VelDir.linear.y =bubble_diff.linear.y/bubble_distance;
  VelDir.angular.z =bubble_diff.angular.z/bubble_distance;

  // if next bubble outside this one we will always be able to break fast enough
  if(bubble_distance > band.at(target_bub_num).expansion )
    return v_max_curr_bub;


  // next bubble center inside this bubble - take into account restrictions on next bubble
  int next_bub_num = target_bub_num + 1;
  geometry_msgs::Twist dummy_twist;
  v_max_next_bub = getBubbleTargetVel(next_bub_num, band, dummy_twist); // recursive call

  // if velocity at next bubble bigger (or equal) than our velocity - we are on the safe side
  if(v_max_next_bub >= v_max_curr_bub)
    return v_max_curr_bub;


  // otherwise max. allowed vel is next vel + plus possible reduction on the way between the bubble-centers
  delta_vel_max = sqrt(2 * bubble_distance * acc_max_);
  v_max_curr_bub = v_max_next_bub + delta_vel_max;

  return v_max_curr_bub;
}


geometry_msgs::Twist SrlEBandTrajectoryCtrl::getFrame1ToFrame2InRefFrame(const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& ref_frame)
{

  geometry_msgs::Pose2D frame1_pose2D, frame2_pose2D, ref_frame_pose2D;
  geometry_msgs::Pose2D frame1_pose2D_rf, frame2_pose2D_rf;
  geometry_msgs::Twist frame_diff;

  // transform all frames to Pose2d
  PoseToPose2D(frame1, frame1_pose2D);
  PoseToPose2D(frame2, frame2_pose2D);
  PoseToPose2D(ref_frame, ref_frame_pose2D);

  // transform frame1 into ref frame
  frame1_pose2D_rf.x = (frame1_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) +
    (frame1_pose2D.y - ref_frame_pose2D.y) * sin(ref_frame_pose2D.theta);
  frame1_pose2D_rf.y = -(frame1_pose2D.x - ref_frame_pose2D.x) * sin(ref_frame_pose2D.theta) +
    (frame1_pose2D.y - ref_frame_pose2D.y) * cos(ref_frame_pose2D.theta);
  frame1_pose2D_rf.theta = frame1_pose2D.theta - ref_frame_pose2D.theta;
  frame1_pose2D_rf.theta = angles::normalize_angle(frame1_pose2D_rf.theta);
  // transform frame2 into ref frame
  frame2_pose2D_rf.x = (frame2_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) +
    (frame2_pose2D.y - ref_frame_pose2D.y) * sin(ref_frame_pose2D.theta);
  frame2_pose2D_rf.y = -(frame2_pose2D.x - ref_frame_pose2D.x) * sin(ref_frame_pose2D.theta) +
    (frame2_pose2D.y - ref_frame_pose2D.y) * cos(ref_frame_pose2D.theta);
  frame2_pose2D_rf.theta = frame2_pose2D.theta - ref_frame_pose2D.theta;
  frame2_pose2D_rf.theta = angles::normalize_angle(frame2_pose2D_rf.theta);

  // get differences
  frame_diff.linear.x = frame2_pose2D_rf.x - frame1_pose2D_rf.x;
  frame_diff.linear.y = frame2_pose2D_rf.y - frame1_pose2D_rf.y;
  frame_diff.linear.z = 0.0;
  frame_diff.angular.x = 0.0;
  frame_diff.angular.y = 0.0;
  frame_diff.angular.z = frame2_pose2D_rf.theta - frame1_pose2D_rf.theta;
  // normalize angle
  frame_diff.angular.z = angles::normalize_angle(frame_diff.angular.z);

  return frame_diff;
}

geometry_msgs::Twist SrlEBandTrajectoryCtrl::getFrame1ToFrame2InRefFrameNew(const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& ref_frame)
{

  double x1 = frame1.position.x - ref_frame.position.x;
  double y1 = frame1.position.y - ref_frame.position.y;
  double x2 = frame2.position.x - ref_frame.position.x;
  double y2 = frame2.position.y - ref_frame.position.y;
  double yaw_ref = tf::getYaw(ref_frame.orientation);

  double x_diff = x2 - x1;
  double y_diff = y2 - y1;
  double theta_diff = atan2(y_diff, x_diff);

  // Now project this vector on to the reference frame
  double rotation = angles::normalize_angle(yaw_ref);
  double x_final = x_diff * cos(rotation) + y_diff * sin(rotation);
  double y_final = - x_diff * sin(rotation) + y_diff * cos(rotation);

  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = x_final;
  twist_msg.linear.y = y_final;
  twist_msg.angular.z = angles::normalize_angle(theta_diff - yaw_ref);

  return twist_msg;
}


geometry_msgs::Twist SrlEBandTrajectoryCtrl::transformTwistFromFrame1ToFrame2(const geometry_msgs::Twist& curr_twist,
    const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2)
{
  geometry_msgs::Pose2D frame1_pose2D, frame2_pose2D;
  geometry_msgs::Twist tmp_transformed;
  double delta_ang;

  tmp_transformed = curr_twist;

  // transform all frames to Pose2d
  PoseToPose2D(frame1, frame1_pose2D);
  PoseToPose2D(frame2, frame2_pose2D);

  // get orientation diff of frames
  delta_ang = frame2_pose2D.theta - frame1_pose2D.theta;
  delta_ang = angles::normalize_angle(delta_ang);

  // tranform twist
  tmp_transformed.linear.x = curr_twist.linear.x * cos(delta_ang) + curr_twist.linear.y * sin(delta_ang);
  tmp_transformed.linear.y = -curr_twist.linear.x * sin(delta_ang) + curr_twist.linear.y * cos(delta_ang);

  return tmp_transformed;
}


geometry_msgs::Twist SrlEBandTrajectoryCtrl::limitTwist(const geometry_msgs::Twist& twist)
{
  geometry_msgs::Twist res = twist;

  //make sure to bound things by our velocity limits
  double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
  double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
  if (lin_overshoot > 1.0)
  {
    res.linear.x /= lin_overshoot;
    res.linear.y /= lin_overshoot;
    // keep relations
    res.angular.z /= lin_overshoot;
  }

  //we only want to enforce a minimum velocity if we're not rotating in place
  if(lin_undershoot > 1.0)
  {
    res.linear.x *= lin_undershoot;
    res.linear.y *= lin_undershoot;
    // we cannot keep relations here for stability reasons
  }

  if (fabs(res.angular.z) > max_vel_th_)
  {
    double scale = max_vel_th_/fabs(res.angular.z);
    //res.angular.z = max_vel_th_ * sign(res.angular.z);
    res.angular.z *= scale;
    // keep relations
    res.linear.x *= scale;
    res.linear.y *= scale;
  }

  if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);
  // we cannot keep relations here for stability reasons

  //we want to check for whether or not we're desired to rotate in place
  if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_)
  {
    if (fabs(res.angular.z) < min_in_place_vel_th_)
      res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);

    res.linear.x = 0.0;
    res.linear.y = 0.0;
  }

  ROS_DEBUG("Angular command %f", res.angular.z);
  return res;
}


}
