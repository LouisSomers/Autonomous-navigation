#ifndef OTTER_GUIDANCE_H_
#define OTTER_GUIDANCE_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <datmo/TrackArray.h>
#include <vector>

namespace otter_coverage
{

class Guidance
{
public:
  Guidance();
  ~Guidance();

private:
  void newWaypoint(const geometry_msgs::PoseStamped& waypoint);
  void newPath(const nav_msgs::Path& path);
  // For Datmo
  void trackingCenter(const datmo::TrackArray& track);
  void getTrackedTarget(double x, double y,float& track_x, float& track_y,std::vector<double>& trackedX,std::vector<double>& trackedY,std::vector<double>& vel_trackedX,std::vector<double>& vel_trackedY, std::vector<int>& vect_id, std::vector<double>& predX,std::vector<double>& predY);
  void drawPrediction_guidance();
  void drawtrajectory_guidance();
  
  void followPath(double x, double y, double psi);
  double dist(double x0, double y0, double x1, double y1) const;

  nav_msgs::Path m_path;
  
  datmo::TrackArray track_center;

  ros::Publisher m_controllerPub;
  ros::Publisher m_pub_prediction_guidance;
  ros::Publisher m_pub_trajectory_guidance;
 
  //ros::Publisher predictorPub;

  // lookahead distance
  double DELTA = 0.5;

  // time-varying lookahead distance
  double delta_max = 4.0;
  double delta_min = 1.0;
  double delta_k = 1.0;

  // circle of acceptance
  double R = 1.0;

  double m_maxSpeed;
  double m_maxSpeedTurn;
  double m_minSpeed;
  std::vector<int> vect_id_center;
  std::vector<double> vect_tracked_centerX;
  std::vector<double> vect_tracked_centerY;
  std::vector<double> velX;
  std::vector<double> velY;
  std::vector<double> predictionsX;
  std::vector<double> predictionsY;


};

} // namespace otter_coverage

#endif
