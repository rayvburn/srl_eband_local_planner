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
#define TRANS_VEL_ABS_LIMIT  1.1
#define ROT_VEL_ABS_LIMIT 1.57;
tf::TransformListener* tf_listener;

namespace srl_eband_local_planner{

  using std::min;
  using std::max;


  SrlEBandTrajectoryCtrl::SrlEBandTrajectoryCtrl() : costmap_ros_(NULL), initialized_(false), band_set_(false), visualization_(false) {}


  SrlEBandTrajectoryCtrl::SrlEBandTrajectoryCtrl(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false), band_set_(false), visualization_(false)
  {
    // initialize planner
    initialize(name, costmap_ros);
    tf_listener = new tf::TransformListener();
    // Initialize pid object (note we'll be further clamping its output)
    pid_.initPid(1, 0, 0, 10, -10);
  }


  SrlEBandTrajectoryCtrl::~SrlEBandTrajectoryCtrl() {}


  void SrlEBandTrajectoryCtrl::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {

    // check if trajectory controller is already initialized
    if(!initialized_)
    {
      // create Node Handle with name of plugin (as used in move_base for loading)
      ros::NodeHandle node_private("~/" + name);

      // read parameters from parameter server
      node_private.param("max_vel_lin", max_vel_lin_, 0.75);
      node_private.param("max_vel_th", max_vel_th_, 1.0);

      node_private.param("min_vel_lin", min_vel_lin_, 0.1);
      node_private.param("min_vel_th", min_vel_th_, 0.0);

      node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);
      node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);

      node_private.param("xy_goal_tolerance", tolerance_trans_, 0.02);
      node_private.param("yaw_goal_tolerance", tolerance_rot_, 0.04);
      node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

      node_private.param("k_prop", k_p_, 4.0);
      node_private.param("k_damp", k_nu_, 3.5);

      node_private.param("Ctrl_Rate", ctrl_freq_, 10.0); // TODO retrieve this from move base parameters

      node_private.param("max_acceleration", acc_max_, 0.5);
      node_private.param("virtual_mass", virt_mass_, 0.75);

      node_private.param("max_translational_acceleration", acc_max_trans_, 0.5);
      node_private.param("max_rotational_acceleration", acc_max_rot_, 1.5);

      node_private.param("rotation_correction_threshold", rotation_correction_threshold_, 0.5);

      // diffferential drive parameters
      node_private.param("differential_drive", differential_drive_hack_, true);
      node_private.param("k_int", k_int_, 0.005);
      node_private.param("k_diff", k_diff_, -0.005);
      node_private.param("bubble_velocity_multiplier", bubble_velocity_multiplier_, 2.0);
      node_private.param("rotation_threshold_multiplier", rotation_threshold_multiplier_, 1.0); //0.75);
      node_private.param("disallow_hysteresis", disallow_hysteresis_, true); //0.75);

      node_private.param("Ts", ts_ , 0.1);
      node_private.param("Kv_one", k_one_, 1.0);
      node_private.param("Kv_two", k_two_, 1.0);
      node_private.param("B", b_, 0.15);
      node_private.param("smoothed_eband", smoothed_eband_, true);
      node_private.param("lookahed", lookahed_, 2);

      // Ctrl_rate, k_prop, max_vel_lin, max_vel_th, tolerance_trans, tolerance_rot, min_in_place_vel_th
      in_final_goal_turn_ = false;

      // copy adress of costmap and Transform Listener (handed over from move_base)
      costmap_ros_ = costmap_ros;


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
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }


  void SrlEBandTrajectoryCtrl::setVisualization(boost::shared_ptr<SrlEBandVisualization> target_visual)
  {
    target_visual_ = target_visual;

    visualization_ = true;
  }

  bool SrlEBandTrajectoryCtrl::setBand(const std::vector<Bubble>& elastic_band)
  {
    elastic_band_ = elastic_band;
    band_set_ = true;

    // create local variables
    std::vector<geometry_msgs::PoseStamped> tmp_plan;

    // adapt plan to band
    tmp_plan.clear();
    plan_.clear();

    double old_x = 0;
    double old_y = 0;
    ROS_DEBUG("Elastic band received in frame %s", elastic_band_[0].center.header.frame_id.c_str());

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




    return true;
  }


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
    theta_curr_ = set_angle_to_range(tf::getYaw(odometry.pose.pose.orientation),0);

    return true;
  }

  // Return the angular difference between the direction we're pointing
  // and the direction we want to move in
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
      // if(DEB_INFO_>0)
      //   ROS_INFO("Transform Pose in Local Planner, in Frame %s", planner_frame_.c_str());

      tf::StampedTransform transform;

      try{

          // will transform data in the goal_frame into the planner_frame_
         tf_listener->waitForTransform( planner_frame_, init_pose.header.frame_id, ros::Time::now(), ros::Duration(0.40));
         tf_listener->lookupTransform(  planner_frame_, init_pose.header.frame_id, ros::Time::now(), transform);

      }
      catch(tf::TransformException){

          ROS_ERROR("Failed to transform the given pose in the Planner frame_id, planner frame %s, inititpose frame %s", planner_frame_.c_str(), init_pose.header.frame_id.c_str());
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

      tf::quaternionTFToMsg(result.getRotation(), res.pose.orientation);

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
      if((fabs(bubble_diff.linear.x) <= 0.6 * tolerance_trans_ &&
          fabs(bubble_diff.linear.y) <= 0.6 * tolerance_trans_) ||
          in_final_goal_turn_) {
        // Calculate orientation difference to goal orientation (not captured in bubble_diff)
        double robot_yaw = tf::getYaw(elastic_band_.at(0).center.pose.orientation);
        double goal_yaw = tf::getYaw(elastic_band_.at((int)elastic_band_.size() - 1).center.pose.orientation);
        float orientation_diff = angles::normalize_angle(goal_yaw - robot_yaw);
        if (fabs(orientation_diff) > tolerance_rot_) {
          in_final_goal_turn_ = true;
          ROS_DEBUG("Performing in place rotation for goal (diff): %f", orientation_diff);
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
        }
        command_provided = true;
      }
    }

    // Check 2 - If we reach here, it means we need to use our PID controller to
    // move towards the next bubble
    if (!command_provided) {

      // Look for current robot pose
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
      theta_curr_ = set_angle_to_range(tf::getYaw(transform.getRotation()), 0);



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
      /// need to trasform path points in odom frame
      geometry_msgs::PoseStamped p;
      p.header = elastic_band_.at(ind_band).center.header;
      p.pose.position.x =  elastic_band_[ind_band].center.pose.position.x;
      p.pose.position.y =  elastic_band_[ind_band].center.pose.position.y;
      p.pose.orientation = elastic_band_[ind_band].center.pose.orientation;
      geometry_msgs::PoseStamped pt = transformPose(p);

      double vy_feed = 0;
      double vx_feed = 0;

      if(smoothed_eband_){

        err_y = -( plan_[ind_band].pose.position.y -(y_curr_+b_*sin(theta_curr_)));
        err_x = -( plan_[ind_band].pose.position.x -(x_curr_+b_*cos(theta_curr_)));

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


      vsig=fabs(feedback_v)/TRANS_VEL_ABS_LIMIT;
      wsig=fabs(feedback_w)/ROT_VEL_ABS_LIMIT;

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

          feedback_v = feedback_w;
          feedback_w = feedback_w;

      }

      if(sig==vsig){

          feedback_v = feedback_v*TRANS_VEL_ABS_LIMIT;
          feedback_w = feedback_w/sig;

      }



      if(sig==wsig){

          feedback_v=feedback_v/sig;
          feedback_w=feedback_w*ROT_VEL_ABS_LIMIT;

      }


      double forward_sign = -2 * (bubble_diff.linear.x < 0) + 1;

      double max_vel_lin = max_vel_lin_;
      if (distance_from_goal < 0.75f) {
        max_vel_lin = (max_vel_lin < 0.3) ? 0.15 : max_vel_lin / 2;
      }

      double linear_velocity = feedback_v;
      // linear_velocity *= cos(bubble_diff.angular.z); //decrease while turning
      if (fabs(linear_velocity) > max_vel_lin_) {
          linear_velocity = forward_sign * max_vel_lin_;
      } else if (fabs(linear_velocity) < min_vel_lin_) {
          linear_velocity = forward_sign * min_vel_lin_;
    }

      // robot_cmd.linear.x = feedback_v;
      // robot_cmd.angular.z = feedback_w;

      robot_cmd.linear.x = linear_velocity;
      robot_cmd.angular.z = feedback_w;


      command_provided = true;
      curr_control_index_++;

    }

    twist_cmd = robot_cmd;
    ROS_DEBUG("Final command: %f, %f", twist_cmd.linear.x, twist_cmd.angular.z);
    return true;



  }




  bool SrlEBandTrajectoryCtrl::getTwistDifferentialDrive(geometry_msgs::Twist& twist_cmd, bool& goal_reached) {
    goal_reached = false;

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
          fabs(bubble_diff.linear.y) <= 0.6 * tolerance_trans_) ||
          in_final_goal_turn_) {
        // Calculate orientation difference to goal orientation (not captured in bubble_diff)
        double robot_yaw = tf::getYaw(elastic_band_.at(0).center.pose.orientation);
        double goal_yaw = tf::getYaw(elastic_band_.at((int)elastic_band_.size() - 1).center.pose.orientation);
        float orientation_diff = angles::normalize_angle(goal_yaw - robot_yaw);
        if (fabs(orientation_diff) > tolerance_rot_) {
          in_final_goal_turn_ = true;
          ROS_DEBUG("Performing in place rotation for goal (diff): %f", orientation_diff);
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
        if (fabs(robot_cmd.angular.z) > max_vel_th_) { // limit max rotation
          robot_cmd.angular.z = rotation_sign * max_vel_th_;
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
      if (distance_from_goal < 0.75f) {
        max_vel_lin = (max_vel_lin < 0.3) ? 0.15 : max_vel_lin / 2;
      }

      double linear_velocity = velocity_multiplier * max_vel_lin;
      linear_velocity *= cos(bubble_diff.angular.z); //decrease while turning
      if (fabs(linear_velocity) > max_vel_lin_) {
        linear_velocity = forward_sign * max_vel_lin_;
      } else if (fabs(linear_velocity) < min_vel_lin_) {
        linear_velocity = forward_sign * min_vel_lin_;
      }

      // Select an angular velocity (based on PID controller)
      double error = bubble_diff.angular.z;
      double rotation_sign = -2 * (bubble_diff.angular.z < 0) + 1;
      double angular_velocity = k_p_ * error;
      if (fabs(angular_velocity) > max_vel_th_) {
        angular_velocity = rotation_sign * max_vel_th_;
      } else if (fabs(angular_velocity) < min_vel_th_) {
        angular_velocity = rotation_sign * min_vel_th_;
      }

      ROS_DEBUG("Selected velocity: lin: %f, ang: %f",
          linear_velocity, angular_velocity);

      robot_cmd.linear.x = linear_velocity;
      robot_cmd.angular.z = angular_velocity;
      command_provided = true;
    }

    twist_cmd = robot_cmd;
    ROS_DEBUG("Final command: %f, %f", twist_cmd.linear.x, twist_cmd.angular.z);
    return true;
  }


  bool SrlEBandTrajectoryCtrl::getTwist(geometry_msgs::Twist& twist_cmd, bool& goal_reached)
  {
    goal_reached = false;
    if (differential_drive_hack_) {
      // return getTwistDifferentialDrive(twist_cmd, goal_reached);
      return getTwistUnicycle(twist_cmd, goal_reached);
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
    ang_pseudo_dist = bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
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


    ang_pseudo_dist = control_deviation.angular.z * getCircumscribedRadius(*costmap_ros_);
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
        ang_pseudo_dist = next_bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
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
          ang_pseudo_dist1 = bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
          ang_pseudo_dist2 = next_bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);
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
    ang_pseudo_dist = control_deviation.angular.z * getCircumscribedRadius(*costmap_ros_);
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
      const double vel = pid_.computeCommand(angular_diff, ros::Duration(1/ctrl_freq_));
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
    ang_pseudo_dist = desired_velocity.angular.z * getCircumscribedRadius(*costmap_ros_);
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
    last_vel_.linear.x = last_vel_.linear.x + acc_desired.linear.x / ctrl_freq_;
    last_vel_.linear.y = last_vel_.linear.y + acc_desired.linear.y / ctrl_freq_;
    last_vel_.angular.z = last_vel_.angular.z + acc_desired.angular.z / ctrl_freq_;


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
    angle_to_pseudo_vel = bubble_diff.angular.z * getCircumscribedRadius(*costmap_ros_);

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
