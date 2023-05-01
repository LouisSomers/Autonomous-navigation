#ifndef PARTITION_BINN_H_
#define PARTITION_BINN_H_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <vector>
#include <datmo/TrackArray.h>


class PartitionBinn {
 public:
  enum CellStatus { Unknown, Free, Blocked };

  struct Cell {
    Cell() : isCovered(false), status(Unknown),predicted(false), x(0.0) {}
    bool isCovered;
    CellStatus status;
    bool predicted;
    double x;  // neural activity
  };
  struct Point {
    int l;
    int k;
  };
  PartitionBinn();
  PartitionBinn(ros::NodeHandle nh);

  void initialize(double x0, double y0, double x1, double y1, double rc,
                  double scanRange);

  void trackingCenter_p(const datmo::TrackArray& track);
  void getTrackedTarget_p(double x, double y,float& track_x, float& track_y,std::vector<double>& trackedX,std::vector<double>& trackedY,std::vector<double>& vel_trackedX,std::vector<double>& vel_trackedY, std::vector<int>& vect_id, std::vector<double>& predX,std::vector<double>& predY);
  datmo::TrackArray track_center;
  void block_cells(double x, double y);
  void drawPartition();

  void update(const nav_msgs::OccupancyGrid& map, double x, double y);

  void gridToWorld(int l, int k, double& xc, double& yc);

  void worldToGrid(double xc, double yc, int& l, int& k);

  std::vector<std::vector<Cell>> getCells() { return m_cells; }

  // added this below for DATMO
  
  void nextWaypointNeeded(int l, int k, bool IsCovered){
    m_cells[l][k].isCovered = IsCovered;
  }

  bool predictioncovered(int l, int k){
    return m_cells[l][k].predicted;
  }

  void setpredictioncovered(int l, int k, bool predictor){
    m_cells[l][k].predicted = predictor;
  }
  void setbacktofree(int l, int k){ 
    m_cells[l][k].predicted = false;
    m_cells[l][k].status = Free;
  }

  //


  void setCellStatus(int l, int k, CellStatus status) {
    m_cells[l - 1][k - 1].status = status;
  }

  CellStatus getCellStatus(int l, int k) {
    return m_cells[l - 1][k - 1].status;
  }

  void setCellCovered(int l, int k, bool isCovered) {
    m_cells[l - 1][k - 1].isCovered = isCovered;
  }


  bool isCellCovered(int l, int k) { return m_cells[l - 1][k - 1].isCovered; }

  void setCellValue(int l, int k, double x) { m_cells[l - 1][k - 1].x = x; }

  double getCellValue(int l, int k) { return m_cells[l - 1][k - 1].x; }

  void getNeighbors(int l, int k, double dist, std::vector<Point>& neighbors);

  bool hasCompleteCoverage();

 private:
  /**
   * @brief given the column l and the row k, calculates
   * the center coordinates of the circle [xc, yc].
   * @param l the column 1<=l<=m
   * @param k the row 1<=k<=n[l]
   * @param xc
   * @param yc
   */
  void gridToLocal(int l, int k, double& xc, double& yc);

  /**
   * @brief given the cartesian coordinates [xc, yc], calculates the column l
   * and row k in the grid
   * @param xc
   * @param yc
   * @param l the column 1<=l<=m
   * @param k the row 1<=k<=n[l]
   */
  void localToGrid(double xc, double yc, int& l, int& k);

  CellStatus calculateStatus(const nav_msgs::OccupancyGrid& map, int l, int k);

  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  


  bool m_initialized;

  double m_x0;
  double m_y0;
  double m_x1;
  double m_y1;
  double m_rc;
  double m_Xw;
  double m_Yw;

  int m_m;               // columns
  std::vector<int> m_n;  // rows

  double m_scanRange;  // in meters

  std::vector<std::vector<Cell>> m_cells;  // m_cells[column][row]
  std::vector<int> vect_id_center;
  std::vector<double> vect_tracked_centerX;
  std::vector<double> vect_tracked_centerY;
  std::vector<double> velX;
  std::vector<double> velY;
  std::vector<double> predictionsX;
  std::vector<double> predictionsY;
};

#endif
