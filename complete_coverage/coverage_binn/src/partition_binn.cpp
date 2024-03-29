#include <cmath>
#include <coverage_binn/partition_binn.h>
#include <visualization_msgs/MarkerArray.h>
#include <datmo/TrackArray.h>


PartitionBinn::PartitionBinn() {}

PartitionBinn::PartitionBinn(ros::NodeHandle nh) : m_initialized(false)
{
  ROS_INFO("PartitionBinn constructed.");
  //subscribers not needed

  ros::Subscriber track_pSub = nh.subscribe("datmo/box_kf",10,&PartitionBinn::trackingCenter_p, this);
 // ros::Subscriber own_posSub = nh.subscribe("startPose")
  
  // publishers
  m_nh = nh;
  m_pub = m_nh.advertise<visualization_msgs::MarkerArray>("cell_partition", 1);
}



void PartitionBinn::initialize(double x0, double y0, double x1, double y1,
                               double rc, double scanRange)
{
  // TODO: rotate coordinate system? so that rectangle doesn't need to be
  // aligned with x and y axis in map frame.

  ROS_INFO("PartitionBinn initialized.");

  m_initialized = true;

  m_x0 = x0;
  m_y0 = y0;
  m_x1 = x1;
  m_y1 = y1;
  m_rc = rc;
  m_scanRange = scanRange;

  // Find the size of the workspace
  m_Xw = x1 - x0;
  m_Yw = y1 - y0;

  // Find number of columns m
  if (std::fmod(m_Xw / (3.0 / 2.0 * rc), 1.0) <= 2.0 / 3.0)
  {
    m_m = static_cast<int>(std::floor(m_Xw / (3.0 / 2.0 * rc))) + 1;
  }
  else if (std::fmod(m_Xw / (3.0 / 2.0 * rc), 1.0) > 2.0 / 3.0)
  {
    m_m = static_cast<int>(std::floor(m_Xw / (3.0 / 2.0 * rc))) + 2;
  }

  // Find number of rows nl in each column
  std::vector<int> n;
  for (int l = 1; l <= m_m; l++)
  {
    if (std::fmod(m_Yw / (std::sqrt(3.0) * rc), 1.0) <= 1.0 / 2.0)
    {
      n.push_back(static_cast<int>(std::floor(m_Yw / (std::sqrt(3.0) * rc))) +
                  1);
    }
    else if (std::fmod(m_Yw / (std::sqrt(3.0) * rc), 1.0) > 1.0 / 2.0)
    {
      n.push_back(static_cast<int>(std::floor(m_Yw / (std::sqrt(3.0) * rc))) +
                  1 + (l % 2));
    }
  }
  m_n = n;

  // Initialize cells
  std::vector<std::vector<Cell>> cells;
  for (int l = 1; l <= m_m; l++)
  {
    std::vector<Cell> column;
    for (int k = 1; k <= m_n[l - 1]; k++)
    {
      Cell cell;
      column.push_back(cell);
    }
    cells.push_back(column);
  }
  m_cells = cells;
}

// for the DATMO node but not needed !
void PartitionBinn::trackingCenter_p(const datmo::TrackArray& track) { track_center = track; }

void PartitionBinn::getTrackedTarget_p(double x, double y,float& track_x, float& track_y,std::vector<double>& trackedX,std::vector<double>& trackedY,std::vector<double>& vel_trackedX,std::vector<double>& vel_trackedY, std::vector<int>& vect_id, std::vector<double>& predX,std::vector<double>& predY){
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
     // ROS_INFO_STREAM("beta and alpha:"<< beta <<"," << alpha);
     // ROS_INFO_STREAM("id begin and end andd mean velocity:" << vect_id[end-1] << "," <<vect_id[end-queue] << "," << vel_mean);
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
      //  ROS_INFO_STREAM("x_mean, y_mean:"<< x_mean <<"," << y_mean);
      //  ROS_INFO_STREAM("vel_mean:"<< vel_mean);

        predX.clear();
        predY.clear();
        double dist_obstacle;
        dist_obstacle = sqrt((x-x_mean)*(x-x_mean)+(y-y_mean)*(y-y_mean));
        if (dist_obstacle <= 25 && vel_mean >=0){
          for (int i = 1; i <= queue; i = i + 1){
            double predicted_x, predicted_y;
            predicted_x =x_mean+vel_x_mean*i*4/2;
            predicted_y =y_mean+vel_y_mean*i*4/2;
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

void PartitionBinn::block_cells(double x, double y){
  

  float track_x, track_y;
  getTrackedTarget_p(x,y,track_x, track_y,vect_tracked_centerX,vect_tracked_centerY,velX,velY,vect_id_center,predictionsX,predictionsY);
  if (predictionsX.empty()){
   // double tttt = track_center.tracks[0].odom.pose.pose.position.x;
  
    ROS_INFO_STREAM("Target not found");
  }
  else{
    for (int i = 0; i <= predictionsX.size(); i = i + 1){
      int l_pred,k_pred;
      worldToGrid(predictionsX[i],predictionsY[i], l_pred, k_pred);
      
      m_cells[l_pred][k_pred].status = Blocked;
    }

    ROS_INFO_STREAM("Target found");

  }

  
}



// end for the datmo 


void PartitionBinn::drawPartition()
{
  visualization_msgs::MarkerArray ma;
  int id = 0;
  for (int l = 1; l <= m_m; l++)
  {
    for (int k = 1; k <= m_n[l - 1]; k++)
    {
      double x;
      double y;
      gridToWorld(l, k, x, y);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "shapes";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.scale.x = m_rc * 2;
      marker.scale.y = m_rc * 2;
      marker.scale.z = 0.01;

      if (m_cells[l - 1][k - 1].status == Free)
      {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
      }
      else if (m_cells[l - 1][k - 1].status == Unknown)
      {
        // Same as background color in rviz
        marker.color.r = 48.0 / 255.0;
        marker.color.g = 48.0 / 255.0;
        marker.color.b = 48.0 / 255.0;
      }
      if (m_cells[l - 1][k - 1].isCovered)
      {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
      }
      if (m_cells[l - 1][k - 1].status == Blocked)
      {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
      }
      marker.color.a = 0.3f;

      marker.lifetime = ros::Duration(0.0);
      ma.markers.push_back(marker);
    }
  }
  m_pub.publish(ma);
}

void PartitionBinn::getNeighbors(int l, int k, double dist,
                                 std::vector<Point>& neighbors)
{
  int maxCellDistance = std::ceil(dist / (1.74 * m_rc)); // 1.74 >= sqrt(3)

  double x0, y0;
  gridToWorld(l, k, x0, y0);

  for (int i = -maxCellDistance; i <= maxCellDistance; i++)
  {
    for (int j = -maxCellDistance; j <= maxCellDistance; j++)
    {
      if (l + i < 1 || l + i > m_m || k + j < 1 || k + j > m_n[l + i - 1])
      {
        continue;
      }
      double x1, y1;
      gridToWorld(l + i, k + j, x1, y1);
      if (std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2)) <= dist)
      {
        neighbors.push_back({l + i, k + j});
      }
    }
  }
}

// TODO: consider using costmap_2d from navigation to get smaller maps
void PartitionBinn::update(const nav_msgs::OccupancyGrid& map, double x,
                           double y)
{
  // Update the status of cells based on the map received in a region around the
  // position [x,y]

  drawPartition();

  

  // Current cell
  int l;
  int k;
  worldToGrid(x, y, l, k);
  //block_cells(x,y);

  // Calculate status for all nearby cells
  std::vector<Point> neighbors;
  getNeighbors(l, k, m_scanRange, neighbors);
 
  for (auto neighbor : neighbors)
  {
    m_cells[neighbor.l - 1][neighbor.k - 1].status =
        calculateStatus(map, neighbor.l, neighbor.k);
  }
}

unsigned int mapIndex(unsigned int x, unsigned int y, unsigned int width)
{
  return y * width + x;
}

PartitionBinn::CellStatus
PartitionBinn::calculateStatus(const nav_msgs::OccupancyGrid& map, int l, int k)
{
  /* A cell in the partition contains many cells in the map. All
   * map cells in a partition cell are free => partition cell is free
   */

  // Find position of partition cell in map frame
  double xWorld;
  double yWorld;
  gridToWorld(l, k, xWorld, yWorld);

  // Find x and y in map of lower right corner of surrounding square
  unsigned int xMap = static_cast<unsigned int>(
      ((xWorld - m_rc) - map.info.origin.position.x) / map.info.resolution);
  unsigned int yMap = static_cast<unsigned int>(
      ((yWorld - m_rc) - map.info.origin.position.y) / map.info.resolution);

  // Check if all map cells in the partition cell is free
  bool unknown = false;
  double xOrigin = map.info.origin.position.x;
  double yOrigin = map.info.origin.position.y;
  double mapRes = map.info.resolution;
  for (unsigned int i = 0; i * mapRes < 2 * m_rc; i++)
  {
    for (unsigned int j = 0; j * mapRes < 2 * m_rc; j++)
    {
      unsigned int m = mapIndex(xMap + i, yMap + j, map.info.width);

      // Map cell not in partition cell (circle shape)
      if (std::sqrt(std::pow((xMap + i) * mapRes + xOrigin - xWorld, 2) +
                    std::pow((yMap + j) * mapRes + yOrigin - yWorld, 2)) > m_rc)
      {
        continue;
      }

      // Map not big enough
      if (xMap + i >= map.info.width || yMap + j >= map.info.height)
      {
        unknown = true;
        continue;
      }


      // Any map cell with occupancy probability > 50
      if (map.data[m] > 50)
      {
        return Blocked;
      }
      
       

      // Any unknown map cell
      if (map.data[m] < 0)
      {
        unknown = true;
      }
    }
  }

  // No blocked cells, but some are unknown.
  if (unknown && getCellStatus(l, k) == Unknown)
    return Unknown;

  // Partially blocked cell
  if (unknown && getCellStatus(l, k) == Blocked)
    return Blocked;

  if (predictioncovered(l,k) == true)
    return Blocked;

  return Free;
}

void PartitionBinn::gridToWorld(int l, int k, double& xc, double& yc)
{
  double xLocal;
  double yLocal;
  gridToLocal(l, k, xLocal, yLocal);
  xc = xLocal + m_x0;
  yc = yLocal + m_y0;
}

void PartitionBinn::worldToGrid(double xc, double yc, int& l, int& k)
{
  double xLocal = xc - m_x0;
  double yLocal = yc - m_y0;
  localToGrid(xLocal, yLocal, l, k);
}

void PartitionBinn::gridToLocal(int l, int k, double& xc, double& yc)
{
  if (l < 1 || l > m_m || k < 0 || k > m_n[l - 1])
    ROS_ERROR("gridToLocal: index out of bounds.");

  if (l % 2 == 1)
  {
    xc = (3.0 / 2.0 * l - 1.0) * m_rc;
    yc = (k - 1.0) * std::sqrt(3) * m_rc;
  }
  else
  { // l % 2 == 0
    xc = (3.0 / 2.0 * l - 1.0) * m_rc;
    yc = (k - 1.0 / 2.0) * std::sqrt(3) * m_rc;
  }
}

// TODO: Not accurate, circles are treated as squares. Maybe do better.
void PartitionBinn::localToGrid(double xc, double yc, int& l, int& k)
{
  if (xc < 0 || xc > m_Xw || yc < 0 || yc > m_Yw)
    ROS_ERROR("localToGrid: index out of bounds.");

  // Find column

  l = static_cast<int>(
          std::lround((std::max(xc - 0.5 * m_rc, 0.0)) / (3.0 / 2.0 * m_rc))) +
      1;

  // Find row
  if (l % 2 == 0)
  {
    k = static_cast<int>(
            std::lround((std::max(yc - std::sqrt(3.0) / 2 * m_rc, 0.0)) /
                        (std::sqrt(3.0) * m_rc))) +
        1;
  }
  else
  { // l % 2 == 1
    k = static_cast<int>(
            std::lround((std::max(yc, 0.0)) / (std::sqrt(3.0) * m_rc))) +
        1;
  }
}

bool PartitionBinn::hasCompleteCoverage()
{
  // Any non-covered free cells?
  for (auto column : m_cells)
  {
    for (auto cell : column)
    {
      if (!cell.isCovered && cell.status == Free)
      {
        return false;
      }
    }
  }
  return true;
}
