/*/
 * Copyright (c) 2015 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Wed Jul 29 2015
 * Luigi Palmieri, added Local Path generation and its visualization, 21 Jan 2016
 */

// defining constants
#define PREDICTD_HUMAN_PUB_TOPIC "predicted_human_poses"
#define PREDICT_SERVICE_NAME "/human_pose_prediction/predict_2d_human_poses"
#define CURR_TRAJECTORY_PUB_TOPIC "predicted_robot_trajectory"

#define ALPHA_MAX 2.09 // (2*M_PI/3) radians, angle between robot heading and inverse of human heading
#define D_LOW 0.7 // meters, minimum distance for compatibility measure
#define D_HIGH 10.0 // meters, maximum distance for compatibility measure
#define BETA 1.57 // meters, angle from robot front to discard human for collision in comaptibility calculations
#define MIN_SCALE 0.05 // minimum scaling of velocities that is always allowed regardless if humans are too near
#define PREDICT_TIME 2.0 // seconds, time for predicting human and robot position, before checking compatibility

#define MESSAGE_THROTTLE_PERIOD 4.0 // seconds

#define HUMAN_RADIUS 0.28 // meters

#include <srl_eband_local_planner/context_cost_function.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hanp_local_planner
{
    // empty constructor and destructor
    ContextCostFunction::ContextCostFunction() {}
    ContextCostFunction::~ContextCostFunction() {}

    void ContextCostFunction::initialize(std::string global_frame, tf2_ros::Buffer* tf)
    {
        ros::NodeHandle private_nh("~/");
        predict_humans_client_ = private_nh.serviceClient<hanp_prediction::HumanPosePredict>(PREDICT_SERVICE_NAME);
        // no way to obtain the actual frame_id of poses predicted by the service above, see transformHumanPoses
        private_nh.param<std::string>("predicted_humans_frame_id", predicted_humans_frame_id_, "odom");

        // initialize variables
        global_frame_ = global_frame;
        tf_ = tf;

        predict_human_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(PREDICTD_HUMAN_PUB_TOPIC, 1);
        marker_trajectory_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(CURR_TRAJECTORY_PUB_TOPIC, 1);

    }

    bool ContextCostFunction::prepare()
    {
        // set default parameters
        setParams(ALPHA_MAX, D_LOW, D_HIGH, BETA, MIN_SCALE, PREDICT_TIME, false, false);

        return true;
    }

    void ContextCostFunction::setParams(double alpha_max, double d_low, double d_high, double beta,
        double min_scale, double predict_time, bool publish_predicted_human_markers, bool publish_curr_traj)
    {
        alpha_max_ = alpha_max;
        d_low_ = d_low;
        d_high_ = d_high;
        beta_ = beta;
        min_scale_ = min_scale;
        predict_time_ = predict_time;
        publish_predicted_human_markers_ = publish_predicted_human_markers;
        publish_curr_trajectory_ = publish_curr_traj;

        ROS_DEBUG_NAMED("context_cost_function", "context-cost function parameters set: "
        "alpha_max=%f, d_low=%f, d_high=%f, beta=%f, min_scale=%f, predict_time=%f",
        alpha_max_, d_low_, d_high_, beta_, min_scale_, predict_time_);
    }

    /// Generate a trajectory given the current position of the robot and the pair of (v,w).
    void ContextCostFunction::generateTrajectory(double x, double y, double theta, double v, double w,
          base_local_planner::Trajectory& traj, bool dir,  bool not_pub){


         //create a potential trajectory
        traj.resetPoints();
        traj.xv_ = v;
        traj.yv_ = 0;
        traj.thetav_ = w;
        traj.cost_ = -1.0;

        double xold = x;
        double yold = y;
        double thetaold = theta;
        double ts = 0.1;

        double vi = v;
        double wi = w;
        // TODO: further tests needed if backward motion flip translational velocity
        // if(dir){
        //   vi = -vi;
        // }

        int num_steps = int( predict_time_ / ts );

        for(int i=0; i<num_steps; i++){
          double xi = xold + vi*ts*cos(thetaold);
          double yi = yold + vi*ts*sin(thetaold);
          double thetai = thetaold + wi*ts;
          // adding point to the Trajectory
          traj.addPoint(xi, yi, thetai);
          // bookkeeping old pose
          xold = xi;
          yold = yi;
          thetaold = thetai;
        }

        ROS_DEBUG_NAMED("context_cost_function", "Trajectory created with %d points", (int)traj.getPointsSize());
        if(publish_curr_trajectory_ && not_pub){
          publishTrajectory(traj);
        }

        return;
    }

    bool ContextCostFunction::publishTrajectory(base_local_planner::Trajectory &traj){

      int marker_id = 0;
      int n_traj_points = (int)traj.getPointsSize();
      visualization_msgs::MarkerArray trajectoryMarkers_array_msg;
      trajectoryMarkers_array_msg.markers.resize(n_traj_points);
      for (int i=0; i<n_traj_points; i++){
        double xi=0;
        double yi=0;
        double thi=0;
        traj.getPoint(i, xi, yi, thi);
        trajectoryMarkers_array_msg.markers[i].header.frame_id = global_frame_;
        trajectoryMarkers_array_msg.markers[i].header.stamp = ros::Time();
        trajectoryMarkers_array_msg.markers[i].id = marker_id++;
        trajectoryMarkers_array_msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
        trajectoryMarkers_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        trajectoryMarkers_array_msg.markers[i].scale.x = 0.4;
        trajectoryMarkers_array_msg.markers[i].scale.y = 0.4;
        trajectoryMarkers_array_msg.markers[i].scale.z = 0.01;
        trajectoryMarkers_array_msg.markers[i].scale.z = 0.01;
        trajectoryMarkers_array_msg.markers[i].color.a = 1.0;
        trajectoryMarkers_array_msg.markers[i].color.r = 0.0;
        trajectoryMarkers_array_msg.markers[i].color.g = 1.0;
        trajectoryMarkers_array_msg.markers[i].color.b = 0.0;
        trajectoryMarkers_array_msg.markers[i].pose.position.x = xi;
        trajectoryMarkers_array_msg.markers[i].pose.position.y = yi;
      }

      marker_trajectory_pub_.publish(trajectoryMarkers_array_msg);
      return true;

    }

    // abuse this function to give sclae with with the trajectory should be truncated
    double ContextCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
    {
        // TODO: discard humans, if information is too old

        hanp_prediction::HumanPosePredict predict_srv;
        double traj_size = traj.getPointsSize();
        std::vector<double> predict_times;
        for(double i = 1.0; i <= traj_size; ++i)
        {
            predict_times.push_back(predict_time_ * (i / traj_size));
        }
        predict_srv.request.predict_times = predict_times;
        predict_srv.request.type = hanp_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE;
        if(!predict_humans_client_.call(predict_srv))
        {
            ROS_DEBUG_THROTTLE_NAMED(MESSAGE_THROTTLE_PERIOD, "context_cost_function",
                "failed to call %s service, is prediction server running?", PREDICT_SERVICE_NAME);
            return 1.0;
        }

        ROS_DEBUG_NAMED("context_cost_function", "received %d predicted humans",
            predict_srv.response.predicted_humans_poses.size());

        // transform humans to the global_frame_
        std::vector<hanp_prediction::PredictedPoses> transformed_humans;
        for (auto human : predict_srv.response.predicted_humans_poses)
        {
            transformed_humans.push_back(transformHumanPoses(human, predicted_humans_frame_id_));
        }
        ROS_DEBUG_NAMED("context_cost_function", "transformied %d humans to %s frame",
            transformed_humans.size(), global_frame_.c_str());

        if(publish_predicted_human_markers_)
        {
            // delete all previous markers
            predicted_humans_markers_.markers.clear();
            visualization_msgs::Marker delete_human;
            delete_human.action = 3; // visualization_msgs::Marker::DELETEALL
            predicted_humans_markers_.markers.push_back(delete_human);
            predict_human_pub_.publish(predicted_humans_markers_);

            // create new markers
            int marker_id = 0;
            predicted_humans_markers_.markers.clear();

            for(auto transformed_human : transformed_humans)
            {
                for(auto transformed_human_pose : transformed_human.poses)
                {
                    visualization_msgs::Marker predicted_human;
                    predicted_human.header.frame_id = global_frame_;
                    predicted_human.header.stamp = ros::Time();
                    predicted_human.id = marker_id++;
                    predicted_human.type = visualization_msgs::Marker::CYLINDER;
                    predicted_human.action = visualization_msgs::Marker::ADD;
                    predicted_human.scale.x = HUMAN_RADIUS;
                    predicted_human.scale.y = HUMAN_RADIUS;
                    predicted_human.scale.z = 0.01;
                    predicted_human.color.a = 1.0;
                    predicted_human.color.r = 0.0;
                    predicted_human.color.g = 0.0;
                    predicted_human.color.b = 1.0;
                    predicted_human.lifetime = ros::Duration(predict_time_);
                    predicted_human.pose.position.x = transformed_human_pose.pose.pose.position.x;
                    predicted_human.pose.position.y = transformed_human_pose.pose.pose.position.y;
                    predicted_humans_markers_.markers.push_back(predicted_human);
                }
            }
            predict_human_pub_.publish(predicted_humans_markers_);

            ROS_DEBUG_NAMED("context_cost_function", "published predicted humans");
        }

        // temporary variables for future robot pose, and compatibility
        double rx, ry, rtheta, d_p, alpha, compatibility;
        auto point_index_max = traj.getPointsSize();

        for(auto transformed_human : transformed_humans)
        {
            unsigned int point_index = 0;
            do
            {
                // get the future pose of the robot
                traj.getPoint(point_index, rx, ry, rtheta);
                auto future_human_pose = transformed_human.poses[point_index]; //TODO: check index validity
                ROS_DEBUG_NAMED("context_cost_function", "selecting futhre human pose %d of %d",
                    point_index, transformed_human.poses.size());

                // discard human behind the robot
                // TODO: check, maybe it is right to compute the angle of the vector from the robot
                // to the humans
                // atan2(future_human_pose.pose2d.y - rx,future_human_pose.pose2d.x - rx))
                auto a_p = fabs(
                    angles::shortest_angular_distance(
                        rtheta,
                        atan2(
                            ry - future_human_pose.pose.pose.position.y,
                            rx - future_human_pose.pose.pose.position.x
                        )
                    )
                );

                if (a_p < beta_)
                {
                    ROS_DEBUG_NAMED("context_cost_function", "discarding human (%d)"
                        " future pose (%u) for compatibility calculations",
                        transformed_human.id, point_index);
                    continue;
                }

                // calculate distance of robot to person
                d_p = hypot(
                    rx - future_human_pose.pose.pose.position.x, ry
                    - future_human_pose.pose.pose.position.y
                ) - HUMAN_RADIUS;
                ROS_DEBUG_NAMED("context_cost_function", "rx=%f, ry=%f, hx=%f, hy=%f, d_p=%f, a_p=%f",
                    rx, ry, future_human_pose.pose.pose.position.x, future_human_pose.pose.pose.position.y, d_p, a_p);
                alpha = fabs(angles::shortest_angular_distance(rtheta,
                    angles::normalize_angle_positive(tf2::getYaw(future_human_pose.pose.pose.orientation)) - M_PI));
                // ROS_DEBUG_NAMED("context_cost_function", "rtheta=%f, h_inv_theta=%f, alpha=%f",
                // rtheta, angles::normalize_angle_positive(future_human_pose.pose2d.theta) - M_PI, alpha);

                // check compatibility
                compatibility = getCompatabilty(d_p, alpha);

                ROS_DEBUG_NAMED("context_cost_function", "calculated compatibility %f"
                    " at d_p=%f, alpha=%f point: x=%f, y=%f (%d of %d)", compatibility,
                    d_p, alpha, rx, ry, point_index, traj.getPointsSize()-1);
            }
            // keep calculating, until we find incompatible situation or end of path
            while((++point_index < point_index_max) && (compatibility > 0.0));
            point_index_max = point_index;

            // ROS_DEBUG_NAMED("context_cost_function", "calculated maximum point index %d"
            //     " (out of %d) for human at x=%d, y=%f", point_index_max, traj.getPointsSize(),
            //     future_human_pose.pose2d.x, future_human_pose.pose2d.y);

            // no need to check more when we have to stop
            if (point_index_max == 1)
            {
                return min_scale_;
            }
        }

        auto scaling = (double)(point_index_max - 1) / (double)(traj.getPointsSize() - 1);
        ROS_DEBUG_NAMED("context_cost_function", "returning scale value of %f", std::max(min_scale_, scaling));

        return std::max(min_scale_, scaling);
    }

    double ContextCostFunction::getCompatabilty(double d_p, double alpha)
    {
        if(d_p <= d_low_)
        {
            return 0.0;
        }
        else if(d_p >= d_high_)
        {
            return 1.0;
        }
        else if(alpha >= alpha_max_)
        {
            return 1.0;
        }
        else
        {
            return (((d_p - d_low_) / d_high_) * (alpha / alpha_max_));
        }
    }

    hanp_prediction::PredictedPoses ContextCostFunction::transformHumanPoses(
        hanp_prediction::PredictedPoses& predicted_human, std::string frame_id)
    {
        if(global_frame_ != frame_id)
        {
            hanp_prediction::PredictedPoses transformed_human;

            //transform human pose in global frame
            try
            {
                geometry_msgs::TransformStamped humans_to_global_transform;
                humans_to_global_transform = tf_->lookupTransform(global_frame_, frame_id, ros::Time(0));

                for(auto predicted_pose : predicted_human.poses)
                {
                    // transform position
                    geometry_msgs::Pose transformed_pose;
                    geometry_msgs::Pose human_pose;
                    human_pose.position.x = predicted_pose.pose.pose.position.x;
                    human_pose.position.y = predicted_pose.pose.pose.position.y;
                    human_pose.position.z = predicted_pose.pose.pose.position.z;
                    human_pose.orientation.x = predicted_pose.pose.pose.orientation.x;
                    human_pose.orientation.y = predicted_pose.pose.pose.orientation.y;
                    human_pose.orientation.z = predicted_pose.pose.pose.orientation.z;
                    human_pose.orientation.w = predicted_pose.pose.pose.orientation.w;

                    tf2::doTransform(human_pose, transformed_pose, humans_to_global_transform);

                    geometry_msgs::PoseWithCovarianceStamped transformed_pose_wcov;
                    transformed_pose_wcov.pose.pose = transformed_pose;
                    // dummy
                    transformed_pose_wcov.pose.covariance[0] = 0.01;
                    transformed_pose_wcov.pose.covariance[1] = 0.01;
                    transformed_pose_wcov.pose.covariance[6] = 0.01;
                    transformed_pose_wcov.pose.covariance[7] = 0.01;
                    transformed_pose_wcov.pose.covariance[14] = 0.01;
                    transformed_pose_wcov.pose.covariance[21] = 0.01;
                    transformed_pose_wcov.pose.covariance[28] = 0.01;
                    transformed_pose_wcov.pose.covariance[35] = 0.01;
                    transformed_human.poses.push_back(transformed_pose_wcov);

                    ROS_DEBUG_NAMED(
                        "context_cost_function",
                        "transformed human pose to %s frame, resulting pose: x=%f, y=%f theta=%f",
                        global_frame_.c_str(),
                        transformed_pose.position.x,
                        transformed_pose.position.y,
                        tf2::getYaw(transformed_human.poses.back().pose.pose.orientation)
                    );
                }

                // save the ID
                transformed_human.id = predicted_human.id;

                // NOTE: in the original implementation, the velocity is kept in the base coordinate system!
                //
                // transform the velocity - simply rotate the vector
                // geometry_msgs::TransformStamped vel_transform;
                // vel_transform = humans_to_global_transform;
                // vel_transform.transform.translation.x = 0.0;
                // vel_transform.transform.translation.y = 0.0;
                // vel_transform.transform.translation.z = 0.0;

                // geometry_msgs::Vector3 vel_linear;
                // vel_linear.x = predicted_human.start_velocity.twist.linear.x;
                // vel_linear.y = predicted_human.start_velocity.twist.linear.y;
                // vel_linear.z = predicted_human.start_velocity.twist.linear.z;

                // geometry_msgs::Vector3 vel_linear_transformed;
                // tf2::doTransform(vel_linear, vel_linear_transformed, vel_transform);

                // geometry_msgs::Quaternion vel_angular = tf2::toMsg(
                //     tf2::Quaternion(predicted_human.start_velocity.twist.angular.z, 0.0, 0.0)
                // );
                // geometry_msgs::Quaternion vel_angular_transformed;
                // tf2::doTransform(vel_angular, vel_angular_transformed, vel_transform);

                // transformed_human.start_velocity.twist.linear.x = vel_linear_transformed.x;
                // transformed_human.start_velocity.twist.linear.y = vel_linear_transformed.y;
                // transformed_human.start_velocity.twist.linear.z = vel_linear_transformed.z;
                // transformed_human.start_velocity.twist.angular.x = 0.0;
                // transformed_human.start_velocity.twist.angular.y = 0.0;
                // transformed_human.start_velocity.twist.angular.z = tf2::getYaw(vel_angular_transformed);
            }
            catch(const tf2::ExtrapolationException &ex)
            {
                ROS_DEBUG("context_cost_function: cannot extrapolate transform");
            }
            catch(const tf2::TransformException &ex)
            {
                ROS_ERROR("context_cost_function: transform failure %s", ex.what());
            }

            return transformed_human;
        }
        else
        {
            return predicted_human;
        }
    }
}
