#include <guidance/guidance.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <usv_msgs/SpeedCourse.h>

#include <datmo/TrackArray.h>
#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Predictor.h>

namespace otter_coverage
{

Guidance::Guidance()
{
  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");

  m_maxSpeed = nhP.param("max_speed", 1.5); //  Was 1.5
  m_maxSpeedTurn = nhP.param("max_speed_turn", 1); // was 0.6
  m_minSpeed = nhP.param("min_speed", 0.4); // was 0.4
// Subscribers
  ros::Subscriber dubinsPathSub =
      nh.subscribe("simple_dubins_path", 1000, &Guidance::newPath, this);
  // Subscribe to the Datmo node    
  ros::Subscriber trackSub = nh.subscribe("datmo/box_kf",10,&Guidance::trackingCenter, this);
// Publishers
  m_controllerPub =
      nh.advertise<usv_msgs::SpeedCourse>("speed_heading", 1000);
  m_pub_prediction_guidance = nh.advertise<visualization_msgs::MarkerArray>("obstacle_prediction_guidance", 1);
  m_pub_trajectory_guidance = nh.advertise<visualization_msgs::MarkerArray>("obstacle_trajectory_guidance", 1);


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ros::Rate rate(10.0);
  while (nh.ok())
  {
    // Get the pose of the robot in the map frame
    geometry_msgs::TransformStamped tfStamped;
    try
    {
      tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                           ros::Duration(0.0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Transform from map to base_link not found: %s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    double x = tfStamped.transform.translation.x;
    double y = tfStamped.transform.translation.y;
    double psi = tf2::getYaw(tfStamped.transform.rotation);

    followPath(x, y, psi);

    float track_x, track_y;
  
    getTrackedTarget(x,y,track_x, track_y,vect_tracked_centerX,vect_tracked_centerY,velX,velY,vect_id_center,predictionsX,predictionsY);
    if (predictionsX.empty()){
      ROS_INFO_STREAM("empty");
    }
    else{
      ROS_INFO_STREAM("filled");
    }
    drawPrediction_guidance();
    drawtrajectory_guidance();
    


    ros::spinOnce();
    rate.sleep();
  }
}

Guidance::~Guidance() {}

void Guidance::newPath(const nav_msgs::Path& path) { m_path = path; }


// for the DATMO node
void Guidance::trackingCenter(const datmo::TrackArray& track) { track_center = track; }

void Guidance::getTrackedTarget(double x, double y,float& track_x, float& track_y,std::vector<double>& trackedX,std::vector<double>& trackedY,std::vector<double>& vel_trackedX,std::vector<double>& vel_trackedY, std::vector<int>& vect_id, std::vector<double>& predX,std::vector<double>& predY){
  for (unsigned int i =0; i<track_center.tracks.size();i++){
    double id, vel_x, vel_y;
    track_x = track_center.tracks[0].odom.pose.pose.position.x;
    track_y = track_center.tracks[0].odom.pose.pose.position.y;
    vel_x = track_center.tracks[0].odom.twist.twist.linear.x;
    vel_y = track_center.tracks[0].odom.twist.twist.linear.y;
    id = track_center.tracks[0].id;
    vect_id.push_back(id);
    trackedX.push_back(track_x);
    trackedY.push_back(track_y);
    vel_trackedX.push_back(vel_x);
    vel_trackedY.push_back(vel_y);
    int end;
    end = vect_id.size();
    double alpha, beta;
    double teller, noemer;
    double x_mean, y_mean;
    double vel_mean, tracked_vel;
    int queue;
    vel_mean = 0; tracked_vel = 0; x_mean = 0; y_mean = 0; alpha = 0; beta=0; teller = 0; noemer = 0;
    queue = 15;
    bool linear_regression = false;
    if (linear_regression == true){

   
      if (vect_id[end-1] == vect_id[end-queue]) {
        for (int i = 1; i <= queue; i = i + 1){
          x_mean = x_mean + trackedX[end-i]/queue;
          y_mean = y_mean + trackedY[end-i]/queue;
          tracked_vel = sqrt(vel_trackedX[end-i]*vel_trackedX[end-i]+vel_trackedY[end-i]*vel_trackedY[end-i]);
          vel_mean = vel_mean + tracked_vel/queue;

        }
        for (int i = 1; i <= queue; i = i + 1){
          teller = teller + (trackedX[end-i]-x_mean)*(trackedY[end-i]-y_mean);
          noemer = noemer + (trackedX[end-i]-x_mean)*(trackedX[end-i]-x_mean);
        }
        beta = teller/noemer;
        alpha = y_mean -(beta*x_mean);

    
      }
      ROS_INFO_STREAM("beta and alpha:"<< beta <<"," << alpha);
      ROS_INFO_STREAM("id begin and end andd mean velocity:" << vect_id[end-1] << "," <<vect_id[end-queue] << "," << vel_mean);
    }
    else{
      if (vect_id[end-1] == vect_id[end-queue]) {
        double vel_x_mean, vel_y_mean;
        vel_x_mean =0;
        vel_y_mean = 0;

        for (int i = 1; i <= queue; i = i + 1){
            x_mean = x_mean + trackedX[end-i]/queue;
            y_mean = y_mean + trackedY[end-i]/queue;
            vel_x_mean = vel_x_mean+ vel_trackedX[end-i]/queue;
            vel_y_mean = vel_y_mean + vel_trackedY[end-i]/queue;
            tracked_vel = sqrt(vel_trackedX[end-i]*vel_trackedX[end-i]+vel_trackedY[end-i]*vel_trackedY[end-i]);
            vel_mean = vel_mean + tracked_vel/queue;

        }
        ROS_INFO_STREAM("x_mean, y_mean:"<< x_mean <<"," << y_mean);
        ROS_INFO_STREAM("vel_mean:"<< vel_mean);

        predX.clear();
        predY.clear();
        double dist_obstacle;
        dist_obstacle = sqrt((x-x_mean)*(x-x_mean)+(y-y_mean)*(y-y_mean));
        if (dist_obstacle <= 45 && vel_mean >=0){
          for (int i = 1; i <= queue; i = i + 1){
            double predicted_x, predicted_y;
            predicted_x =x_mean+vel_x_mean*i;
            predicted_y =y_mean+vel_y_mean*i;
            predX.push_back(predicted_x);
            predY.push_back(predicted_y);
          }
        
        }
        ROS_INFO_STREAM("predicted end:"<< predX[14] <<"," << predY[14]);

  
      }
      
    }
  
    // ROS_INFO_STREAM("center_box_position:"<< track_x << "," << track_y << "," << id);
  }
}

void Guidance::drawPrediction_guidance(){

  visualization_msgs::MarkerArray ma;
  int id = 0;
  for (int l = 1; l <= predictionsX.size(); l++)
  {


    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "shapes";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = predictionsX[l];
    marker.pose.position.y = predictionsY[l];
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.01;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.lifetime = ros::Duration(0.0);
    ma.markers.push_back(marker);
    
  }
  m_pub_prediction_guidance.publish(ma);
}

void Guidance::drawtrajectory_guidance(){

  visualization_msgs::MarkerArray ma;
  int id = 0;
  for (double l = -10; l <= 10 ; l = l+1)
  {


    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "shapes";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 22;
    marker.pose.position.y = l;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.01;
    marker.color.r = 0;
    marker.color.g = 255;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.lifetime = ros::Duration(0.0);
    ma.markers.push_back(marker);
    
  }
  m_pub_trajectory_guidance.publish(ma);
}

// end for the datmo 


void Guidance::followPath(double x, double y, double psi)
// TODO: cuts turns, how to fix?
{

  // Finished?
  if (m_path.poses.size() <= 1)
  {
    usv_msgs::SpeedCourse msg;
    msg.speed = 0.0;
    msg.course = psi;
    m_controllerPub.publish(msg);
    return;
  }

  // Identify closest point on path
  std::vector<geometry_msgs::PoseStamped>::iterator closest;
  double minDist = std::numeric_limits<double>::max();
  for (auto it = m_path.poses.begin(); it != m_path.poses.end(); it++)
  {
    double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                            std::pow(y - it->pose.position.y, 2));
    if (dist < minDist)
    {
      minDist = dist;
      closest = it;
    }
  }

  // Store closest
  geometry_msgs::PoseStamped pose_d = *closest;

  // Erase previous elements
  m_path.poses.erase(m_path.poses.begin(), closest);

  // Path tangential angle
  double gamma_p = tf2::getYaw(pose_d.pose.orientation);

  // Cross-track error
  double y_e = -(x - pose_d.pose.position.x) * std::sin(gamma_p) +
               (y - pose_d.pose.position.y) * std::cos(gamma_p);

  // Time-varying lookahead distance
  double delta_y_e =
      (delta_max - delta_min) * std::exp(-delta_k * std::pow(y_e, 2)) +
      delta_min;
  // if turning => small lookahead distance
  bool isTurning = false;
  if ((closest + 1) != m_path.poses.end())
  {
    double nextAngle = tf2::getYaw((*(closest + 1)).pose.orientation);
    if (std::fabs(gamma_p - nextAngle) > std::numeric_limits<double>::epsilon())
    {
      delta_y_e = delta_min;
      isTurning = true;
    }
  }

  // velocity-path relative angle
  double chi_r = std::atan(-y_e / delta_y_e);

  // desired course angle
  double chi_d = gamma_p + chi_r;

  // calculate error in heading
  double chi_err = chi_d - psi;
  while (chi_err > M_PI)
  {
    chi_err -= 2 * M_PI;
  }
  while (chi_err < -M_PI)
  {
    chi_err += 2 * M_PI;
  }

  // calculate desired speed
  double u = m_maxSpeed * (1 - std::abs(y_e) / 5 - std::abs(chi_err) / M_PI_2);
  u = std::max(u, m_minSpeed);
  if (isTurning)
    u = m_maxSpeedTurn;

  // Publish speed and course to controller
  usv_msgs::SpeedCourse msg;
  msg.speed = u;
  msg.course = chi_d;
  m_controllerPub.publish(msg);

  //ROS_INFO_STREAM("psi_d: " << chi_d << " psi: " << psi);
  //ROS_INFO_STREAM("u_d: " << u);

}

} // namespace otter_coverage
