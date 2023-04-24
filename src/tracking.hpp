/*
 * Copyright (c) 2020, Robobrain.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Konstantinos Konstantinidis */

#include <ros/ros.h>
#include <math.h>       /* atan */
#include <omp.h>      //Multi-threading
#include <vector>
#include <random>
#include <algorithm> // for sort(), min()
#include <chrono>
#include <iostream>
#include <fstream>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <datmo/TrackArray.h>
#include <datmo/Track.h>

#include "cluster.hpp"

typedef std::pair<double, double> Point;
typedef std::vector<double> l_shape;
typedef std::vector<l_shape> l_shapes;
typedef std::vector<Point> pointList;


using namespace std;
// This node segments the point cloud based on the break-point detector algorithm.
// This algorithm is based on "L-Shape Model Switching-Based Precise Motion Tracking 
// of Moving Vehicles Using Laser Scanners.
class Tracking
{
public:
  Tracking();
  ~Tracking();

  void tracking_center()

  tf::TransformListener tf_listener;
private:
 

};
