#include <cmath>
#include <navigation/navigation.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <datmo/Track.h>
#include <geometry_msgs/Pose.h>
//
#include <string>
//

#include <coverage_boustrophedon/DubinInput.h>

//
#include <iostream>
#include <string>
#include <fstream>
//

double pointDistance(double x0, double y0, double x1, double y1)
{
  return std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

navigation::navigation() : m_mapInitialized(false)
{
  ROS_INFO("navigation_node started.");

  ros::NodeHandle nh;
  ros::NodeHandle nhP("~");

  // Get parameters
  m_x0 = nhP.param("x0", -51.0);
  m_y0 = nhP.param("y0", -51.0);
  m_x1 = nhP.param("x1", 100.0);
  m_y1 = nhP.param("y1", 50.0);
  double cellRadius = nhP.param("cell_radius", 2.5); // SET TO 2.5
  double scanRange = nhP.param("scan_range", 20); // set to 12 originally
  m_circleAcceptance = nhP.param("goal_tolerance", 1.0); // set to 3 originally

  // Set up partition. TODO: set up with parameters
  m_partition = Partition_nav(nh);
  m_partition.initialize(m_x0, m_y0, m_x1, m_y1, cellRadius, scanRange);

  // Set up subscribers
  ros::Subscriber mapSub = nh.subscribe("inflated_map", 1000, &navigation::onMapReceived, this);
 // ros::Subscriber datmoSub = nh.subscribe("Track",1000, &navigation::datmolive);


  // Set up publishers
  m_goalPub =
      nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
  m_dubinPub = nh.advertise<coverage_boustrophedon::DubinInput>(
      "simple_dubins_path/input", 1000);

  // Start main loop
  ROS_INFO("Starting main loop.");
  mainLoop(nh);
}

//void navigation::datmolive(const datmo::Track::ConstPtr& data,double& x_moving,double& id_moving)
//{
  //id_moving = data->id;
  //x_moving = msg_track_box_kf.odom.pose.pose.position.x;
//}


void navigation::onMapReceived(const nav_msgs::OccupancyGrid& grid)
{
  if (!m_mapInitialized)
    m_mapInitialized = true;
  m_partition.update(grid, m_pose.x, m_pose.y);
}

void navigation::mainLoop(ros::NodeHandle& nh)
{
  // Set up tf2 transform listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Loop
  ros::Rate rate(5.0);
  while (nh.ok())
  {
    if (!updateRobotPose(tfBuffer))
    {
      ros::Duration(1.0).sleep();
      continue;
    }

    // Do BINN
    BINN();
    std::ofstream fw("c:/home/louis/catkin_ws/src/log/output.txt", std::ofstream::out);
    if (fw.is_open())
    {
      //store array contents to text file
     
      fw << "test" << "\n";
      
      fw.close();
    }
  

    ros::spinOnce();
    rate.sleep();
  }
}

bool navigation::updateRobotPose(const tf2_ros::Buffer& tfBuffer)
{
  geometry_msgs::TransformStamped tfStamped;
  try
  {
    // TODO: use params for frames
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0),
                                         ros::Duration(0.1));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // Get pose of robot in map frame
  m_pose.x = tfStamped.transform.translation.x;
  m_pose.y = tfStamped.transform.translation.y;
  m_pose.yaw = tf2::getYaw(tfStamped.transform.rotation);

  return true;
}

void navigation::BINN()
{
  double xTarget, yTarget;
  int lTarget, kTarget;
  xTarget=60;
  yTarget=0;
  m_partition.worldToGrid(xTarget, yTarget, lTarget, kTarget);

  if (!m_mapInitialized)
    return;

  // Are we finished?
  // TODO: check for free, uncovered cells
  bool IsWaypointCovered;
  m_partition.nextWaypointNeeded(lTarget, kTarget, IsWaypointCovered);
  if (IsWaypointCovered == true)
  {
    ROS_INFO("Waypoint Reached ask Next waypoint");
    return;
  }

  static ros::Time lastTime = ros::Time::now();
  double deltaTime = (ros::Time::now() - lastTime).toSec();
  lastTime = ros::Time::now();

  // Update neural activity of each cell
  // TODO: figure out why deltaTime by itself is too large
  // TODO: doesn't work with higher deltaTime
  evolveNeuralNetwork(0.01);

  double xNext, yNext, yawNext, DistanceToTarget;
  findNextTargetWaypoint(xTarget, yTarget, xNext,yNext,yawNext,DistanceToTarget);

  // Find current position ( the current cell not the exact position)
  int l_current, k_current;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l_current, k_current);
  double xCurrent, yCurrent;
  m_partition.gridToWorld(l_current, k_current, xCurrent, yCurrent); // center of the circle
  // Set current cell as covered, if we pass even a little in it 
  // In that way we know how he navigates
  double cellRadius = 2.5;
  if (pointDistance(m_pose.x, m_pose.y, xCurrent, yCurrent) <
      cellRadius)
  {
    m_partition.setCellCovered(l_current, k_current, true);
  }

  publishGoal(xNext,yNext,yawNext);

  ROS_INFO_STREAM("Current pos: " << m_pose.x << ", " << m_pose.y);
  ROS_INFO_STREAM("Next pos:    " << xNext << ", " << yNext << ", " << yawNext);
  ROS_INFO_STREAM("Goal pos coordinates:"<< xTarget << "," << yTarget << "," << DistanceToTarget);


}


// THIS WORKS NOW ADAPT IT TO AVOID THE OBSTACLE
void navigation::findNextTargetWaypoint(double& xTarget, double& yTarget, double& xNext, double& yNext, double& yawNext, double& DistanceToTarget)
{
  // Get cell position HERE THE CELLS ARE FOUND FOR THE NEXT WAYPOINT
  double angle_change;
  angle_change = 5; // turns always right now check if left is a better option and decide on which way to turn then
  int l, k;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l, k);
  // Get Target position and the cell of the target position
  int lTarget,kTarget;
  m_partition.worldToGrid(xTarget,yTarget,lTarget,kTarget);
  double DistanceToTargetX,DistanceToTargetY;
  DistanceToTargetX = m_pose.x-xTarget;
  DistanceToTargetY = m_pose.y-yTarget;
  DistanceToTarget = sqrt(DistanceToTargetX*DistanceToTargetX+DistanceToTargetY*DistanceToTargetY);

  // will look in front of the track to see if there are obstacles
  int Next_cellx, Next_celly, Next_cellx2, Next_celly2;
  double angle_of_line;
  
  if (xTarget-m_pose.x != 0) // else you move on a line parallel with the yaxis --> 90° or 270°
  {
    if (yTarget >= m_pose.y){ // you move towards the North (up in the axis)
      angle_of_line = atan((yTarget-m_pose.y)/(xTarget-m_pose.x));
    }
    else{
      angle_of_line = atan((yTarget-m_pose.y)/(xTarget-m_pose.x))+ M_PI;
    }
  }
  else {
    if (yTarget >= m_pose.y){
      angle_of_line = M_PI/2;
    }
    else {
      angle_of_line = M_PI/2+ M_PI;
    }
  }
  // a point in front (1 cell)
  Next_cellx = m_pose.x+6*cos(angle_of_line);
  Next_celly = m_pose.y+6*sin(angle_of_line);

  // a point further away (2 cells)
  Next_cellx2 = m_pose.x+12*cos(angle_of_line);
  Next_celly2 = m_pose.y+12*sin(angle_of_line);

  int lNext_cell, kNext_cell, lNext_cell2, kNext_cell2;
  m_partition.worldToGrid(Next_cellx,Next_celly,lNext_cell,kNext_cell);
  m_partition.worldToGrid(Next_cellx2,Next_celly2,lNext_cell2,kNext_cell2);

  // iIf there are no obstacles the USV moves further towards the target
  if (m_partition.getCellStatus(lNext_cell,kNext_cell) == Partition_nav::Free && m_partition.getCellStatus(lNext_cell2,kNext_cell2) == Partition_nav::Free)
  { 
    xNext = xTarget;
    yNext = yTarget;
    double yawNeeded;
    m_dubin.getTargetHeading(m_pose.x, m_pose.y, m_pose.yaw, xTarget,
                                yTarget, yawNeeded);
    yawNext = yawNeeded;
 
  }
  // There is an obstruction on the way to the point, so need to go around it
  else { 
      //right while loop
      int i = 0;
      double ii,iii;
      double Next_cellx2_blocked_right, Next_celly2_blocked_right, Next_cellx2_blocked_left,Next_celly2_blocked_left;
      double Next_cellx_blocked_right, Next_celly_blocked_right,Next_cellx_blocked_left,Next_celly_blocked_left;
      int lNext_cell_right,kNext_cell_right;
      int lNext_cell2_right,kNext_cell2_right;
      do {
        ii = i*M_PI/180;
        // The calculation for the turning to the right
        Next_cellx_blocked_right = m_pose.x+6*(cos(angle_of_line)+cos(ii));
        Next_celly_blocked_right = m_pose.y+6*(sin(angle_of_line)+sin(ii));
       
        Next_cellx2_blocked_right = m_pose.x+12*(cos(angle_of_line)+cos(ii));
        Next_celly2_blocked_right= m_pose.y+12*(sin(angle_of_line)+sin(ii));
        
        m_partition.worldToGrid(Next_cellx_blocked_right,Next_celly_blocked_right,lNext_cell_right,kNext_cell_right);
        m_partition.worldToGrid(Next_cellx2_blocked_right,Next_celly2_blocked_right,lNext_cell2_right,kNext_cell2_right);

        i += angle_change;
      }
      while (m_partition.getCellStatus(lNext_cell_right,kNext_cell_right) == Partition_nav::Blocked || m_partition.getCellStatus(lNext_cell2_right,kNext_cell2_right) == Partition_nav::Blocked);
      //left while loop
      i = 0;
      int lNext_cell_left,kNext_cell_left;
      int lNext_cell2_left,kNext_cell2_left;
      do {
        iii = -i*M_PI/180;
        // The calculation for the turning to the left
        Next_cellx_blocked_left = m_pose.x+6*(cos(angle_of_line)+cos(iii));
        Next_celly_blocked_left = m_pose.y+6*(sin(angle_of_line)+sin(iii));
       
        Next_cellx2_blocked_left = m_pose.x+12*(cos(angle_of_line)+cos(iii));
        Next_celly2_blocked_left= m_pose.y+12*(sin(angle_of_line)+sin(iii));
        
        m_partition.worldToGrid(Next_cellx_blocked_left,Next_celly_blocked_left,lNext_cell_left,kNext_cell_left);
        m_partition.worldToGrid(Next_cellx2_blocked_left,Next_celly2_blocked_left,lNext_cell2_left,kNext_cell2_left);

        i += angle_change;
      }
      while (m_partition.getCellStatus(lNext_cell_left,kNext_cell_left) == Partition_nav::Blocked || m_partition.getCellStatus(lNext_cell2_left,kNext_cell2_left) == Partition_nav::Blocked);
      // Decide to turn left or right here
      if (ii <= abs(iii))
      {
        lNext_cell= lNext_cell_right; // The 
        kNext_cell = kNext_cell_right;
      }
      else{
        lNext_cell= lNext_cell_left;
        kNext_cell = kNext_cell_left;
      }
      double Nextpointx, Nextpointy;
      m_partition.gridToWorld(lNext_cell,kNext_cell,Nextpointx,Nextpointy);
      xNext = Nextpointx;//Nextxx; // could also use the grid to avoid the obstacle
      yNext=  Nextpointy;//Nextyy;
      double yawNeeded2;
      m_dubin.getTargetHeading(m_pose.x, m_pose.y, m_pose.yaw, xNext,
                                yNext, yawNeeded2);
      yawNext = yawNeeded2;
  
  }
}

//void navigation::driftthroughwind()
// based on the wind it is possible that the course to steer must be adapted
//void navigation::setNextWaypoint(double& xTarget, double& yTarget double& XNextwaypoint, double& YnextWaypoint)

double navigation::calculateI(Partition_nav::CellStatus status, bool covered,
                                double x, double y)
{
  // Scaling factor for target priorities: 0 < lambda <= 1
  // TODO: Prioritize targets smarter, maybe do this with score function
  // instead.
  double lambda = 1;
  // lambda -= 0.9 * (1 - (x + m_x0) / (m_x1 - m_x0));

  if (status == Partition_nav::Free && !covered)
  { // target
    return lambda * m_E;
  }
  else if (status == Partition_nav::Blocked)
  { // obstacle
    return -m_E;
  }
  else
  {
    return 0.0;
  }
}

double navigation::calculateWeightSum(int l, int k)
{
  std::vector<Partition_nav::Point> neighbors;
  getNeighbors2(l, k, neighbors);

  double weightSum = 0.0;
  for (auto nb : neighbors)
  {
    if (nb.l < 1 || nb.l > m_partition.getCells().size() || nb.k < 1 ||
        nb.k > m_partition.getCells()[nb.l - 1].size())
    {
      continue;
    }

    weightSum += calculateWeight(l, k, nb.l, nb.k) *
                 std::max(m_partition.getCellValue(nb.l, nb.k), 0.0);
  }

  return weightSum;
}

// TODO: can be calculated offline and stored
double navigation::calculateWeight(int l0, int k0, int l1, int k1)
{
  double x0, y0;
  m_partition.gridToWorld(l0, k0, x0, y0);
  double x1, y1;
  m_partition.gridToWorld(l1, k1, x1, y1);

  return m_mu / std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

void navigation::getNeighbors(int l, int k,
                                std::vector<Partition_nav::Point>& neighbors)
{
  neighbors.push_back({l + 1, k});
  neighbors.push_back({l + 1, k + 1});
  neighbors.push_back({l + 1, k - 1});
  neighbors.push_back({l - 1, k});
  neighbors.push_back({l - 1, k + 1});
  neighbors.push_back({l - 1, k - 1});
  neighbors.push_back({l + 2, k});
  neighbors.push_back({l - 2, k});
  neighbors.push_back({l, k + 1});
  neighbors.push_back({l, k - 1});
  if (l % 2 == 1)
  {
    neighbors.push_back({l + 1, k - 2});
    neighbors.push_back({l - 1, k - 2});
  }
  else
  { // l % 2 == 0
    neighbors.push_back({l + 1, k + 2});
    neighbors.push_back({l - 1, k + 2});
  }
}

void navigation::getNeighbors2(int l, int k,
                                 std::vector<Partition_nav::Point>& neighbors)
{
  neighbors.push_back({l + 1, k});
  neighbors.push_back({l - 1, k});
  neighbors.push_back({l, k + 1});
  neighbors.push_back({l, k - 1});
  if (l % 2 == 1)
  {
    neighbors.push_back({l + 1, k - 1});
    neighbors.push_back({l - 1, k - 1});
  }
  else
  { // l % 2 == 0
    neighbors.push_back({l + 1, k + 1});
    neighbors.push_back({l - 1, k + 1});
  }
}

void navigation::findNextCell(int& lNext, int& kNext, double& yawNext)
{
  // Get cell position HERE THE CELLS ARE FOUND FOR THE NEXT WAYPOINT
  int l, k;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l, k);

  // Next pos has to be among the neighbors
  std::vector<Partition_nav::Point> neighbors;
  getNeighbors2(l, k, neighbors);

  // Consider current cell as well
  neighbors.push_back({l, k});

  // Find neighbor with highest score
  Partition_nav::Point best = {l, k};
  double maxScore = std::numeric_limits<double>::lowest();
  for (auto nb : neighbors)
  {
    if (nb.l < 1 || nb.l > m_partition.getCells().size() || nb.k < 1 ||
        nb.k > m_partition.getCells()[nb.l - 1].size())
    {
      continue;
    }

    // Current cell position
    double xCurrent, yCurrent;
    m_partition.gridToWorld(l, k, xCurrent, yCurrent);

    // Target position
    
    double xTarget, yTarget;
    m_partition.gridToWorld(nb.l, nb.k, xTarget, yTarget);

    // Target reachable?
    double yawTarget;
    if (m_dubin.getTargetHeading(m_pose.x, m_pose.y, m_pose.yaw, xTarget,
                                 yTarget, yawTarget))
    {
      // Get the score
      double score = scoreFunction(m_partition.getCellValue(nb.l, nb.k),
                                   m_pose.yaw, yawTarget);
      if (score > maxScore)
      {
        maxScore = score;
        best = nb;
        yawNext = yawTarget;
      }
    }
  }

  lNext = best.l;
  kNext = best.k;
}


double navigation::scoreFunction(double neuralActivity, double yaw,
                                   double targetYaw)
{
  if (yaw < 0)
    yaw += 2 * M_PI;
  if (targetYaw < 0)
    targetYaw += 2 * M_PI;
  double diff = std::fabs(targetYaw - yaw);
  if (diff > M_PI)
    diff = 2 * M_PI - diff;

  return (1 - diff / M_PI) * m_lambda * neuralActivity +
         (1 - m_lambda) * neuralActivity;
}


void navigation::evolveNeuralNetwork(double deltaTime)
{
  auto cells = m_partition.getCells();

  for (int l = 1; l <= cells.size(); l++)
  {
    auto column = cells[l - 1];
    for (int k = 1; k <= column.size(); k++)
    {
      double xCell, yCell;
      m_partition.gridToWorld(l, k, xCell, yCell);
      double I = calculateI(m_partition.getCellStatus(l, k),
                            m_partition.isCellCovered(l, k), xCell, yCell);

      double weightSum = calculateWeightSum(l, k);

      double x = m_partition.getCellValue(l, k);

      double xDot = -m_A * x + (m_B - x) * (std::max(I, 0.0) + weightSum) -
                    (m_D + x) * std::max(-I, 0.0);

      x += xDot * deltaTime;
      m_partition.setCellValue(l, k, x);
    }
  }
}


void navigation::publishGoal(double x, double y, double yaw)
{
  // Publish goal
  geometry_msgs::PoseStamped goalPose;
  goalPose.header.stamp = ros::Time::now();
  goalPose.header.frame_id = "map";
  goalPose.pose.position.x = x;
  goalPose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goalPose.pose.orientation.x = q.x();
  goalPose.pose.orientation.y = q.y();
  goalPose.pose.orientation.z = q.z();
  goalPose.pose.orientation.w = q.w();
  m_goalPub.publish(goalPose);

  // Current pose
  geometry_msgs::PoseStamped startPose;
  startPose.header.stamp = ros::Time::now();
  startPose.header.frame_id = "map";
  startPose.pose.position.x = m_pose.x;
  startPose.pose.position.y = m_pose.y;
  startPose.pose.position.z = 0.0;
  q.setRPY(0, 0, m_pose.yaw);
  startPose.pose.orientation.x = q.x();
  startPose.pose.orientation.y = q.y();
  startPose.pose.orientation.z = q.z();
  startPose.pose.orientation.w = q.w();

  // Publish DubinInput
  coverage_boustrophedon::DubinInput di;
  di.header.stamp = ros::Time::now();
  di.header.frame_id = "map";
  di.start = startPose;
  di.end = goalPose;
  m_dubinPub.publish(di);
}
