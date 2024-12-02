#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

// 初始化三维栅格地图
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE]; // 用于记录每个网格单元的状态
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr **[GLX_SIZE]; // 三维指针数组
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k); // 当前网格单元索引
        Vector3d pos = gridIndex2coord(tmpIdx); // 网格索引转换为实际空间坐标
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else {  // 将目标坐标的网格单元,和与目标坐标附近的8个邻域网格单元标记为障碍物
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

/*
  -1 closed list 表示已经访问过的节点
  1  open list 表示待拓展的节点
*/
vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id == -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", (int)visited_nodes.size());
  return visited_nodes;
}

/*
  网格索引转换为实际空间坐标
  索引值偏移0.5,是为了网格的索引定位到网格单元的中心
*/
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

// 三维空间坐标转换为栅格网格索引
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

// 连续三维坐标“圆整”到最近的网格索引位置，然后返回这个位置的实际坐标
Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

// 判断给定网格索引是否空闲（没有障碍物）
inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

/*
  获取当前节点的所有邻居节点，并计算它们之间的边的代价
*/
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        neighborIdx(0) = (currentPtr->index)(0) + dx;
        neighborIdx(1) = (currentPtr->index)(1) + dy;
        neighborIdx(2) = (currentPtr->index)(2) + dz;

        if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
            neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
            neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
          continue;
        }

        if(isOccupied(neighborIdx(0), neighborIdx(1), neighborIdx(2)) || GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]->id==-1){
          continue;
        }

        neighborPtrSets.push_back(
            GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.
  double h;

  int distance[3];
  for(int i=0; i<3; i++){
    distance[i] = abs(node2->index(i)-node1->index(0));
  }
  std::sort(distance, distance+3);
  h = distance[0] + distance[1] + distance[2] + (sqrt(3)-3.0)*distance[0] + (sqrt(2)-2.0)*distance[1];

  if(use_tie_breaker){
    double p = 1/sqrt(pow(distance[0],2)+pow(distance[1],2)+pow(distance[2],2));
    h = h*(1+p);
  }
  return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  /**
   *
   * STEP 1.1:  finish the AstarPathFinder::getHeu
   *
   * **/
  startPtr->fScore = getHeu(startPtr, endPtr);

  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  /**
   *
   * STEP 1.2:  some else preparatory works which should be done before while
   * loop
   *
   * **/

  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.3:  finish the loop
   *
   * **/
  while (!openSet.empty()) {
    // 弹出最小代价f节点
    currentPtr = openSet.begin()->second;
    currentPtr->id = -1;  // 标记为闭集

    // 从开集中弹出
    openSet.erase(openSet.begin());

    // 拓展邻居节点
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

    // 遍历邻居节点
    for (int i=0; i<(int)neighborPtrSets.size(); i++){
      neighborPtr = neighborPtrSets[i];
      tentative_gScore = currentPtr->gScore + edgeCostSets[i];

      if(neighborPtr->id == 0){ // 自由节点
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
        neighborPtr->cameFrom = currentPtr;

        // 存储当前节点在开集中的迭代器
        neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));

        if(neighborPtr->index == goalIdx){
          ros::Time time_2 = ros::Time::now();
          terminatePtr = neighborPtr;
          ROS_WARN("[A*]{success} Time in A* is %f ms, path cost is %f m", (time_2-time_1).toSec()*1000.0, currentPtr->gScore*resolution);
          return;
        }
        else{
          neighborPtr->id = 1;
          continue;
        }
      }
      else if(neighborPtr->id == 1){  // 已在openlist里
        if(neighborPtr->gScore > tentative_gScore){
          neighborPtr->gScore = tentative_gScore;
          neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
          neighborPtr->cameFrom = currentPtr;

          openSet.erase(neighborPtr->nodeMapIt);
          neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
        }
      }
      else{
        continue;
      }
    }
        
  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/
  GridNodePtr tmp_ptr = terminatePtr;
  while(tmp_ptr->cameFrom != NULL){
      gridPath.push_back(tmp_ptr);
      tmp_ptr = tmp_ptr->cameFrom;
  }

  for (auto ptr: gridPath)
    path.push_back(ptr->coord);

  reverse(path.begin(), path.end());

  return path;
}

double AstarPathFinder::disP2L(const Vector3d node1, const Vector3d node2, const Vector3d point){
  Vector3d nu = (point-node1).cross(point-node2);
  Vector3d de = node1-node2;
  double dis = nu.norm()/de.norm();
  
  return dis;
}
/**
 * RDP算法，将由线段组成的曲线抽取为点较少的相似曲线
 * 递归地划分线，其中包含第一个点和最远的点，然后是最远的点和最后一个点
**/
vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/

  if (path.size()<=2){
    subPath = path;
  }
  else{
    const Vector3d first = path[0];
    const Vector3d last = path[path.size()-1];

    int index_dis=0;
    double dis=0;

    for(int i=1; i<(int)path.size()-1; i++){
      double tmp_dis = disP2L(first, last, path[i]);
      if(tmp_dis>dis){
        dis = tmp_dis;
        index_dis = i;
      }
    }
  
    if(dis<path_resolution){
      subPath.push_back(first);
      subPath.push_back(last);
    }
    else{
      vector<Vector3d> recSubPath1 = pathSimplify(vector<Vector3d>(path.begin(), path.begin()+index_dis), path_resolution);
      vector<Vector3d> recSubPath2 = pathSimplify(vector<Vector3d>(path.begin()+index_dis, path.end()), path_resolution);
      subPath.insert(subPath.end(), recSubPath1.begin(), recSubPath1.end()-1);
      subPath.insert(subPath.end(), recSubPath2.begin(), recSubPath2.end());
    }
  }

  return subPath;
}

/**
 * 基于多项式插值来计算每个维度的位置信息
 * 从多项式系数矩阵polyCoeff中计算出某一时刻t对应的轨迹位置
**/
Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/
  Vector3d pos;
  for(int k=0; k<time.size(); k++){
    for(double t=0.0; t<time(k); t+=0.01){
      pos = getPosPoly(polyCoeff, k, t);
      if(isOccupied(coord2gridIndex(pos))){
        return k;
      }
    }
  }

  return unsafe_segment;
}