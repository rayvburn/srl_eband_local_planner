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
* Author: Christian Connette
*********************************************************************/

/*********************************************************************
*  srl_eband_local_planner, local planner plugin for move_base based
*  on the elastic band principle.
*  License for the srl_eband_local_planner software developed
*  during the EU FP7 Spencer Projet: new BSD License.
*
*  Software License Agreement (BSD License)
*
*  Copyright (c) 2015-2016, Luigi Palmieri, University of Freiburg
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
*   * Neither the name of University of Freiburg nor the names of its
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

#ifndef SRL_EBAND_TRAJECTORY_CONTROLLER_H_
#define SRL_EBAND_TRAJECTORY_CONTROLLER_H_

#include <ros/ros.h>
#include <ros/assert.h>

// classes which are part of this package
#include <srl_eband_local_planner/conversions_and_types.h>
#include <srl_eband_local_planner/srl_eband_visualization.h>

// geometry_msg
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Bool.h>
// sensor msg
#include <sensor_msgs/LaserScan.h>

// nav_msgs
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// geometry
#include <angles/angles.h>
#include <tf2_ros/buffer.h>


#include <srl_eband_local_planner/srlEBandLocalPlannerConfig.h>

// PID control library
#include <control_toolbox/pid.h>

#include <srl_eband_local_planner/curvature_properties.h>
#include <srl_eband_local_planner/context_cost_function.h>

#include <srl_eband_local_planner/check_points_on_path.h>

namespace srl_eband_local_planner{

  /**
   * @class EBandPlanner
   * @brief Implements the Elastic Band Method for SE2-Manifold (mobile Base)
   */
  class SrlEBandTrajectoryCtrl{

    public:

      /**
       * @brief Default constructor
       */
      SrlEBandTrajectoryCtrl();

      /**
       * @brief Constructs the elastic band object
       * @param name The name to give this instance of the elastic band local planner
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      SrlEBandTrajectoryCtrl(std::string name, costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf);

      /**
       * @brief  Destructor
       */
      ~SrlEBandTrajectoryCtrl();

      /**
       * @brief Initializes the elastic band class by accesing costmap and loading parameters
       * @param name The name to give this instance of the trajectory planner
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros, tf2_ros::Buffer* tf);

      /**
       * @brief passes a reference to the eband visualization object which can be used to visualize the band optimization
       * @param pointer to visualization object
       */
      void setVisualization(boost::shared_ptr<SrlEBandVisualization> target_visual);

      /**
       * @brief This sets the elastic_band to the trajectory controller
       * @param reference via which the band is passed
       */
      bool setBand(const std::vector<Bubble>& elastic_band);

      /**
       * @brief This sets the robot velocity as provided by odometry
       * @param reference via which odometry is passed
       */
      bool setOdometry(const nav_msgs::Odometry& odometry);

      /**
       * @brief calculates a twist feedforward command from the current band
       * @param refernce to the twist cmd
       */
      bool getTwist(geometry_msgs::Twist& twist_cmd, bool& goal_reached);

      /**
       * @brief calculates a twist feedforward command from the current band for a differential drive robot
       * @param refernce to the twist cmd
       */
      bool getTwistDifferentialDrive(geometry_msgs::Twist& twist_cmd, bool& goal_reached);

      /**
       * @brief calculates a twist feedforward command from the current band for a unicycle robot
       * @param refernce to the twist cmd
       */
      bool getTwistUnicycle(geometry_msgs::Twist& twist_cmd, bool& goal_reached);

      /**
       * @brief compute_sigma
       * @param sigma value
       */
      double compute_sigma(double vsig, double wsig);


      /**
       * @brief set_angle_to_range, set angle alpha in the range [min, min+2*M_PI]
       * @param alpha, the angle to set in the proper range
       * @param min, initial angle of the range
       */
      double set_angle_to_range(double alpha, double min);

      /**
       * @brief transformPose, trasform pose to the planner frame
       */
      geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& init_pose);

      /**
       * @brief checkAccelerationBounds, set the twist so to respect acceleration bounds
       * @param twist_cmd, initial Twist command
       * @return the scaled velocity
       */
      geometry_msgs::Twist checkAccelerationBounds(geometry_msgs::Twist twist_cmd);

      /**
       * @brief callbackDynamicReconfigure, Changes the dynamic parameters
       */
      void callbackDynamicReconfigure(srl_eband_local_planner::srlEBandLocalPlannerConfig &config, uint32_t level);

      /**
       * @brief callbackLaserScanReceived, Used to read laser scans and limit the current maximum velocity
       */
      void callbackLaserScanReceived(
        const sensor_msgs::LaserScanConstPtr& laserscan,
        int& num_points_near_robot_bwd_on,
        int& num_points_near_robot_bwd_off
      );

      void callbackDrivinDirection(const std_msgs::Bool::ConstPtr& msg);

      /**
       * @brief publishLocalPlan, Publishing the final local path
       */
      void publishLocalPlan(base_local_planner::Trajectory local_traj);


    void publishRepairedPlan(std::vector<geometry_msgs::PoseStamped>& plan);



      /**
      * @brief publishRepairedPlan, Publish final plan
      * @return void
      */
      void setCostMap(costmap_2d::Costmap2DROS* costmap_ros);

      /**
      * @brief set limits on the velocity
      * @return void
      */
      void setDifferentialDriveVelLimits(double v, double w);

      /**
      * @brief set limits on the velocity
      * @return void
      */
      void setDifferentialDriveHRIVelLimits(double v, double w);

      /**
      * @brief set limits on the velocity
      * @return void
      */
      void setCollisionStatus(bool collision_warning_front,
                                        bool collision_warning_rear);


      /**
      * @brief set the limits of the Velocity during HRI
      * @return void
      */
      bool limitVelocityHRI(double &curr_max_vel);

    private:

      // pointer to external objects (do NOT delete object)
      costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap
      boost::shared_ptr<SrlEBandVisualization> target_visual_; // pointer to visualization object
      ros::Subscriber sub_front_laser_;
      ros::Subscriber sub_rear_laser_;
      ros::Publisher pub_local_path_;
      ros::NodeHandle nh_;

      dynamic_reconfigure::Server<srl_eband_local_planner::srlEBandLocalPlannerConfig> *dr_server_;
      control_toolbox::Pid pid_;
      double circumscribed_radius_;
      // parameters
      bool differential_drive_on_;
      double k_p_, k_nu_, k_int_, k_diff_, ctrl_freq_;
      double acc_max_, virt_mass_;
      double max_vel_lin_, max_vel_th_, min_vel_lin_, min_vel_th_;
      double min_in_place_vel_th_;
      double in_place_trans_vel_;
      double tolerance_trans_, tolerance_rot_, tolerance_timeout_;
      double acc_max_trans_, acc_max_rot_;
      double rotation_correction_threshold_; // We'll do rotation correction if we're at least this far from the goal
      std::string local_path_topic_;
      double integral_angular_;
      double previous_angular_error_;
      int path_id_;
      // diff drive only parameters
      double bubble_velocity_multiplier_;
      double rotation_threshold_multiplier_;
      bool disallow_hysteresis_;
      bool in_final_goal_turn_;
      bool initial_turn_;

      bool initial_band_;
      double x_initial_band_;
      double y_initial_band_;
      double theta_initial_band_;
      double rot_stopping_turn_on_the_spot_;
      double max_rotational_velocity_turning_on_spot_;
      double trans_vel_goal_;
      double start_to_stop_goal_;
      bool tracker_on_;
      // flags
      bool initialized_, band_set_, visualization_;

      // data
      std::vector<Bubble> elastic_band_;
      geometry_msgs::Twist odom_vel_;
      geometry_msgs::Twist last_vel_;
      geometry_msgs::Pose ref_frame_band_;

      std::vector<geometry_msgs::PoseStamped> plan_;
      std::vector<double> dx_d_ ;
      std::vector<double> dy_d_ ;
      std::string planner_frame_ ;
      double ts_ ;
      double k_two_;
      double k_one_;
      double b_;
      bool smoothed_eband_;
      double x_curr_;
      double y_curr_;
      double theta_curr_;
      int lookahed_;
      int curr_control_index_;
      CurvatureProperties *compute_curvature_properties_;
      double controller_frequency_;
      double curvature_guarding_thrs_;
      bool limit_vel_based_on_curvature_;
      double min_vel_limited_curvature_;
      bool limit_vel_based_laser_points_density_;
      int num_points_front_robot_;
      int num_points_rear_robot_;
      double warning_robot_radius_;
      double warning_robot_angle_;
      bool backward_motion_on_;
      double max_translational_vel_due_to_laser_points_density_;
      double max_vel_th_hri_;
      double max_vel_lin_hri_;
      bool limit_vel_based_on_hri_;
      double old_linear_velocity_;
      double old_angular_velocity_;
      bool limit_acc_;

      std::string rear_laser_topic_;
      std::string front_laser_topic_;
      std::string robot_frame_;
      /// human awareness
      bool human_legibility_on_;

      hanp_local_planner::ContextCostFunction *context_cost_function_;
      check_points_on_path::CheckPointsOnPath *check_laser_on_path_;
      bool laser_points_on_band_;
      double max_path_length_to_check_points_;

      bool collision_warning_front_;
      bool collision_warning_rear_;
      double max_vel_collision_warning_;
      bool limit_vel_collision_warnings_;
      ///@brief defines sign of a double
      inline double sign(double n)
      {
        return n < 0.0 ? -1.0 : 1.0;
      }

      /**
       * @brief Transforms Pose of frame 1 and 2 into reference frame and gets difference of frame 1 and 2
       * @param refernce to pose of frame1
       * @param reference to pose of frame2
       * @param reference to pose of reference frame
       * @return vector from frame1 to frame2 in coordinates of the reference frame
       */
      geometry_msgs::Twist getFrame1ToFrame2InRefFrame(const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& ref_frame);
      geometry_msgs::Twist getFrame1ToFrame2InRefFrameNew(const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& ref_frame);

      /**
       * @param Transforms twist into a given reference frame
       * @param Twist that shall be transformed
       * @param refernce to pose of frame1
       * @param reference to pose of frame2
       * @return transformed twist
       */
      geometry_msgs::Twist transformTwistFromFrame1ToFrame2(const geometry_msgs::Twist& curr_twist, const geometry_msgs::Pose& frame1,
          const geometry_msgs::Pose& frame2);

      /**
       * @brief limits the twist to the allowed range
       * @param reference to unconstrained twist
       * @return twist in allowed range
       */
      geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);


      /**
       * @brief limits the max translational and rotational velocity based on the current curvature of the path
       * @param curr_max_vel, current max velocity
       * @return true if nothing bad happened
       */
      bool limitVelocityCurvature(double &curr_max_vel);

      /**
       * @brief limits the max translational and rotational velocity based on the density of the laser scans
       * @param curr_max_vel, current max velocity
       * @param band_dir, direction of the elastic band
       * @return true if nothing bad happened
       */
      bool limitVelocityDensityLaserPoints(double &curr_max_vel, double band_dir);

      /**
       * @brief limits the max translational  based on the current collision warnings
       * @param curr_max_vel, current max velocity
       * @return true if nothing bad happened
       */
      bool limitVelocityCollisionWarnings(double &curr_max_vel);

      /**
       * @brief gets the max velocity allowed within this bubble depending on size of the bubble and pose and size of the following bubble
       * @param number of the bubble of interest within the band
       * @param band in which the bubble is in
       * @return absolute value of maximum allowed velocity within this bubble
       */
      double getBubbleTargetVel(const int& target_bub_num, const std::vector<Bubble>& band, geometry_msgs::Twist& VelDir);



  };
};
#endif
