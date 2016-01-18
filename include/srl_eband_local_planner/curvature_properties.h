/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Freiburg
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

#include <list>
#include <limits>

using namespace std;

class CurvatureProperties {

public:

    CurvatureProperties() {
      index_last_max_K_ = 0;

    }

    ~CurvatureProperties(){
    }


    int getLastIndexMaxCurvature(){

        return index_last_max_K_;

    }

    double pathMaxCurvature(vector<double> x_v, vector<double> y_v){


        double x1, x2, x3, y1, y2, y3, v1x, v2x, v1y, v2y, v1, v2, k_i;
        double max = std::numeric_limits<double>::max();
        int index_max_K = 0;
        double maxK = 0;

        int traj_size = x_v.size();


        // Handles the empty input path, setting curvature to infinity
        if( traj_size == 0){
            maxK = max;
            return maxK;
        }

        // Handles the input path of length 1 or 2, setting curvature to 0
        if( traj_size<3)
            return 0;


        // We can compute the curvature in all the points of the path
        // except the first and the last one
        for (int i=0; i<(traj_size-2); i++){

            x1 = x_v[i];
            x2 = x_v[i+1];
            x3 = x_v[i+2];

            y1 = y_v[i];
            y2 = y_v[i+1];
            y3 = y_v[i+2];

            // if two points in a row repeat, we skip curvature computation
            if(x1 == x2 && y1 == y2 || x2 == x3 && y2 == y3)
                continue;


            // Infinite curvature in case the path goes a step backwards:
            // p1 - p2 - p1
            if(x1 == x3 && y1 == y3){
                std::cout << "Warning! Undefined curvature! Skipping three steps..."<<endl;
                continue;
            }

            // Normalization of vectors p2 -> p1 and p2 -> p3 to length 1.
            v1x = x1-x2;
            v1y = y1-y2;
            v2x = x3-x2;
            v2y = y3-y2;
            v1 = sqrt(v1x*v1x + v1y*v1y);
            v2 = sqrt(v2x*v2x + v2y*v2y);
            v1x = (0.5 * v1x * (v1+v2))/v1;
            v1y = (0.5 * v1y * (v1+v2))/v1;
            v2x = (0.5 * v2x * (v1+v2))/v2;
            v2y = (0.5 * v2y * (v1+v2))/v2;

            x1 = x2 + v1x;
            y1 = y2 + v1y;
            x3 = x2 + v2x;
            y3 = y2 + v2y;

            // curvature computation
            k_i= 2*fabs((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1)) / (sqrt(( (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1) )*( (x3-x1)*(x3-x1)+(y3-y1)*(y3-y1) )*( (x3-x2)*(x3-x2)+(y3-y2)*(y3-y2) )));


            if(k_i > maxK){
                index_max_K = i;
                maxK = k_i;
              }
        }
        index_last_max_K_ = index_max_K;
        return maxK;


    }


  public:

    int index_last_max_K_;


};
