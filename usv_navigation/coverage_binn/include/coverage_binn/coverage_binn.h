#ifndef COVERAGE_BINN_H_
#define COVERAGE_BINN_H_
#include <iostream>
#include <string>
#include <coverage_binn/partition_binn.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <coverage_binn/simple_dubins_path.h>
#include <tf2_ros/transform_listener.h>
#include <datmo/Track.h>


class CoverageBinn {
 public:
  CoverageBinn();

 private:
  //void datmolive(const datmo::Track::ConstPtr& data, double& x_moving, double& id_moving);
  void onMapReceived(const nav_msgs::OccupancyGrid& grid);
  //void trackingCenter(const datmo::Track& track);
  
  void mainLoop(ros::NodeHandle& nh);
  bool updateRobotPose(const tf2_ros::Buffer& tfBuffer);
  void BINN();
  void evolveNeuralNetwork(double deltaTime);
  double calculateI(PartitionBinn::CellStatus status, bool covered, double x,
					double y);
  double calculateWeightSum(int l, int k);
  double calculateWeight(int l0, int k0, int l1, int k1);
  void getNeighbors(int l, int k, std::vector<PartitionBinn::Point>& neighbors);
  void getNeighbors2(int l, int k,
					 std::vector<PartitionBinn::Point>& neighbors);
  void findNextCell(int& lNext, int& kNext, double& yawNext);

  // added the void next target waypoint in this; 
  void findNextTargetWaypoint(double& xTarget, double& yTarget, double& xNext, double& yNext, double& yawNext, double& DistanceToTarget);
  void trackingCenter_p(const datmo::TrackArray& track);
  void getTrackedTarget_p(double x, double y,float& track_x, float& track_y,std::vector<double>& trackedX,std::vector<double>& trackedY,std::vector<double>& vel_trackedX,std::vector<double>& vel_trackedY, std::vector<int>& vect_id, std::vector<double>& predX,std::vector<double>& predY);
  datmo::TrackArray track_center;
  void block_cells(double x, double y);
  void deblock_cells();
  void drawPrediction();
  void drawReversePrediction();
  //void CoverageBinn::driftthroughwind
  //void CoverageBinn::setNextWaypoint(double& xTarget, double& yTarget double& XNextwaypoint, double& YnextWaypoint)
  //  
  double scoreFunction(double neuralActivity, double yaw, double targetYaw);
  void publishGoal(double x, double y, double yaw);

 
  struct Pose {
    double x;
    double y;
    double yaw;
  };

  bool m_mapInitialized;
  PartitionBinn m_partition;
  Pose m_pose;

  // Partition
  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;

  // Neural network params
  const double m_A = 50.0;
  const double m_B = 0.1;
  const double m_D = 0.1;
  const double m_E = 100.0;
  const double m_mu = 1.0;
  const double m_lambda = 0.1;

  std::vector<int> vect_id_center;
  std::vector<double> vect_tracked_centerX;
  std::vector<double> vect_tracked_centerY;
  std::vector<double> velX;
  std::vector<double> velY;
  std::vector<double> predictionsX;
  std::vector<double> predictionsY;

  // Params
  double m_circleAcceptance = 1.0;

  coverage_binn::SimpleDubinsPath m_dubin;

  ros::Publisher m_goalPub;
  ros::Publisher m_dubinPub;
  ros::Publisher m_pub_prediction;
  ros::Publisher m_pub_reverse_prediction;

};

#endif
