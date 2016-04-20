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


#include <Eigen/Geometry>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include <set>

// geometry_msg
#include <geometry_msgs/PoseStamped.h>


typedef boost::array<double, 2> Array2D;

typedef boost::array<double, 4> Array4D;

typedef boost::array<int, 2> Array2I;

namespace check_points_on_path{

  class CheckPointsOnPath{

  public:

      CheckPointsOnPath();

      ~CheckPointsOnPath();

      Array4D intersectlines(Array4D x1, Array4D x2, Array4D x3, Array4D x4);

      double edist(Array4D v1, Array4D v2);

      std::pair<double, bool> distance2Segment(Array4D x, Array4D xs,
                                        Array4D xe,  Array4D &proj, double &dout);

      bool checkLaserPointInside(double x, double y, double *point_path_distance);

      bool setPathDistance(double p);

      bool setPath(std::vector<geometry_msgs::PoseStamped> local_plan);

      double path_distance_;

      Eigen::MatrixXd *path_;

  };

}
