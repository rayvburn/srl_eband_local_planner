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

#ifndef SRL_EBAND_LOCAL_PLANNER_ROS_H_
#define SRL_EBAND_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// classes wich are parts of this pkg
#include <srl_eband_local_planner/srl_eband_local_planner.h>
#include <srl_eband_local_planner/conversions_and_types.h>
#include <srl_eband_local_planner/srl_eband_visualization.h>
#include <srl_eband_local_planner/srl_eband_trajectory_controller.h>
#include <srl_eband_local_planner/costmap_layers_dyn_rec_handler.h>
// service to handle the costmap_2d layers
#include <srl_eband_local_planner/EnableSocialLayer.h>
#include <srl_eband_local_planner/EnableObstacleLayer.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>

// msgs
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// transforms
#include <angles/angles.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <srl_eband_local_planner/srlEBandLocalPlannerConfig.h>
#include <spencer_tracking_msgs/TrackedPersons.h>

#define EPS 0.01

namespace srl_eband_local_planner{

  /**
   * @class EBandPlannerROS
   * @brief Plugin to the ros base_local_planner. Implements a wrapper for the Elastic Band Method
   */
  class SrlEBandPlannerROS : public nav_core::BaseLocalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      SrlEBandPlannerROS();

      /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the elastic band local planner
       * @param tf A pointer to a transform buffer
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      SrlEBandPlannerROS(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~SrlEBandPlannerROS();

      /**
       * @brief Initializes the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform buffer
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Set the plan that the controller is following; also reset eband-planner
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();


      void callbackDynamicReconfigure(srl_eband_local_planner::srlEBandLocalPlannerConfig &config, uint32_t level);

      /**
      * @brief  Set Driving Direction
      * @return void
      */
      void SetDrivingDirection(const std_msgs::Bool::ConstPtr& msg);

      void readVelocityCB(const geometry_msgs::TwistStamped::ConstPtr& msg);


      bool enableSocialLayer(srl_eband_local_planner::EnableSocialLayer::Request  &req,
               srl_eband_local_planner::EnableSocialLayer::Response &res);

      bool enableObstacleLayer(srl_eband_local_planner::EnableObstacleLayer::Request  &req,
               srl_eband_local_planner::EnableObstacleLayer::Response &res);

      bool setCostmapsLayers();

      void callbackAllTracks(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg);

      void callbackTriggerHRI(const std_msgs::Bool::ConstPtr& msg);



    private:

      // pointer to external objects (do NOT delete object)
      costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap
      costmap_2d::Costmap2DROS* costmap_only_static_;
      costmap_2d::Costmap2DROS* costmap_ros_initial_; ///<@brief pointer to costmap

      tf2_ros::Buffer* tf_; ///<@brief pointer to Transform Buffer

      dynamic_reconfigure::Server<srl_eband_local_planner::srlEBandLocalPlannerConfig> *dr_server_;
      ros::ServiceServer service_enable_social_layer_;
      ros::ServiceServer service_enable_obstacle_layer_;


      // parameters
      double yaw_goal_tolerance_, xy_goal_tolerance_; ///<@brief parameters to define region in which goal is treated as reached
      double rot_stopped_vel_, trans_stopped_vel_; ///<@brief lower bound for absolute value of velocity (with respect to stick-slip behaviour)

      // Topics & Services
      ros::Publisher g_plan_pub_; ///<@brief publishes modified global plan
      ros::Publisher l_plan_pub_; ///<@brief publishes prediction for local commands
      ros::Publisher pub_hri_message_;
      ros::Subscriber odom_sub_; ///<@brief subscribes to the odometry topic in global namespace
      ros::Subscriber sub_current_driving_direction_;
      ros::Subscriber sub_current_measured_velocity_;
      ros::Subscriber sub_tracks_;
      ros::Subscriber sub_trigger_hri_;

      // data
      std::vector<geometry_msgs::PoseStamped> agents_position; ///<  @brief agents
      nav_msgs::Odometry base_odom_;
      std::vector<geometry_msgs::PoseStamped> global_plan_; // plan as handed over from move_base or global planner
      std::vector<geometry_msgs::PoseStamped> transformed_plan_; // plan transformed into the map frame we are working in
      std::vector<int> plan_start_end_counter_; // stores which number start and end frame of the transformed plan have inside the global plan

      // pointer to locally created objects (delete - except for smart-ptrs:)
      boost::shared_ptr<SrlEBandPlanner> eband_;
      boost::shared_ptr<SrlEBandVisualization> eband_visual_;
      boost::shared_ptr<SrlEBandTrajectoryCtrl> eband_trj_ctrl_;

      bool goal_reached_;
      int dir_planning_;
      // flags
      bool initialized_;
      boost::mutex odom_mutex_; // mutex to lock odometry-callback while data is read from topic

      bool optimize_band_;
      int number_tentative_setting_band_;
      // methods
      // methods
      double initial_social_range_;
      costmapLayersDynRecHandler *costmap_layers_handler_; ///< @brief Costmap handler for dynamic reconfiguration
      bool check_costmap_layers_;
      bool collision_error_front_;
      bool collision_error_rear_;
      bool collision_warning_front_;
      bool collision_warning_rear_;
      bool enable_social_layer_;
      bool enable_obstacle_layer_;
      bool robot_still_position_;
      double max_lin_vel_;
      double max_lin_vel_hri_;
      bool trigger_hri_;
      double min_alert_dist_tracks_;
      double max_ang_range_tracks_;
      int cnt_tracks_in_front_;
      std::string hr_message_;
      double time_hri_last_;
      double waiting_time_hri_message_;
      /**
       * @brief Odometry-Callback: function is called whenever a new odometry msg is published on that topic
       * @param Pointer to the received message
       */
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  };
};
#endif
