double CoverageBinn::calculateI(PartitionBinn::CellStatus status, bool covered,
                                double x, double y)
{
  // Scaling factor for target priorities: 0 < lambda <= 1
  // TODO: Prioritize targets smarter, maybe do this with score function
  // instead.
  double lambda = 1;
  // lambda -= 0.9 * (1 - (x + m_x0) / (m_x1 - m_x0));

  if (status == PartitionBinn::Free && !covered)
  { // target
    return lambda * m_E;
  }
  else if (status == PartitionBinn::Blocked)
  { // obstacle
    return -m_E;
  }
  else
  {
    return 0.0;
  }
}

double CoverageBinn::calculateWeightSum(int l, int k)
{
  std::vector<PartitionBinn::Point> neighbors;
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
double CoverageBinn::calculateWeight(int l0, int k0, int l1, int k1)
{
  double x0, y0;
  m_partition.gridToWorld(l0, k0, x0, y0);
  double x1, y1;
  m_partition.gridToWorld(l1, k1, x1, y1);

  return m_mu / std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
}

void CoverageBinn::getNeighbors(int l, int k,
                                std::vector<PartitionBinn::Point>& neighbors)
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

void CoverageBinn::getNeighbors2(int l, int k,
                                 std::vector<PartitionBinn::Point>& neighbors)
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

void CoverageBinn::findNextCell(int& lNext, int& kNext, double& yawNext)
{
  // Get cell position HERE THE CELLS ARE FOUND FOR THE NEXT WAYPOINT
  int l, k;
  m_partition.worldToGrid(m_pose.x, m_pose.y, l, k);

  // Next pos has to be among the neighbors
  std::vector<PartitionBinn::Point> neighbors;
  getNeighbors2(l, k, neighbors);

  // Consider current cell as well
  neighbors.push_back({l, k});

  // Find neighbor with highest score
  PartitionBinn::Point best = {l, k};
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


double CoverageBinn::scoreFunction(double neuralActivity, double yaw,
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