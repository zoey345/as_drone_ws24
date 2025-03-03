#include <Eigen/Dense>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>               /* 修改: 添加PCL PointIndices头文件 */
#include <pcl/search/kdtree.h>              /* 修改: 添加PCL KdTree头文件 */
#include <ros/ros.h>
#include <vector>
#include <functional>                       /* 修改: 添加std::function支持 */

class FrontierEstimationNode {
public:
  FrontierEstimationNode() {
      state_subscriber = node_handle.subscribe("current_state_est", 1, &FrontierEstimationNode::currentStateCallback, this);
      map_subscriber = node_handle.subscribe("octomap_full", 1, &FrontierEstimationNode::mapUpdateCallback, this);
      goal_publisher = node_handle.advertise<geometry_msgs::Point>("frontier_goal", 1);

      node_handle.getParam("/octomap_server/resolution", octomap_resolution);
      // 如果max_distance未设置参数，可在此处设定默认值
      max_distance = 20; /* 修改: 默认设置搜索范围 */
      exploration_timer = node_handle.createTimer(ros::Rate(0.3), &FrontierEstimationNode::explorationTimerCallback, this);
  }

private:
  ros::NodeHandle node_handle;
  ros::Subscriber state_subscriber;
  ros::Subscriber map_subscriber;
  ros::Publisher goal_publisher;
  ros::Timer exploration_timer;

  octomap::OcTree::leaf_bbx_iterator iterator;
  std::shared_ptr<octomap::OcTree> octree{};
  octomap::point3d current_position;
  std::vector<geometry_msgs::Point> frontier_points;
  int octomap_resolution, max_distance; 
  std::vector<pcl::PointIndicesPtr> cluster_indices;

  void currentStateCallback(const nav_msgs::Odometry &msg) {
      current_position = octomap::point3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  }

  void mapUpdateCallback(const octomap_msgs::OctomapConstPtr &msg) {
      octree = std::make_shared<octomap::OcTree>(*dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
  }

  void explorationTimerCallback(const ros::TimerEvent &event) {
      if (!octree)
          return;

      detectFrontiers();
      auto frontier_cloud = generateFrontierCloud();

      if (frontier_cloud->empty()) {
        ROS_WARN("frontier_cloud is empty!");
      }

      // 使用PCL库中的DBSCAN聚类算法替代原Optics聚类
      cluster_indices = dbscanClustering(frontier_cloud, 10.0, 5); /* 修改: 使用DBSCAN聚类，参数eps=10.0, minPts=5，可根据需要调整 */

      auto largest_cluster_cloud = identifyLargestCluster(frontier_cloud);
      if(largest_cluster_cloud->points.empty()){
          ROS_WARN("No valid frontier clusters found.");
          return;
      }
      auto goal = calculateGoal(*largest_cluster_cloud);

      ROS_INFO("Frontier goal point set to: (%f, %f, %f)", goal.x, goal.y, goal.z);
      publishGoal(goal);
  }

  void detectFrontiers() {
      frontier_points.clear();
      octomap::point3d minPt(current_position.x() - max_distance,
                           current_position.y() - max_distance,
                           current_position.z() - max_distance);
      // 注意: 这里对maxPt.x使用std::min与-340.0f的比较，需根据实际情况确认是否合理
      octomap::point3d maxPt(std::min(current_position.x() + max_distance, -340.0f),
                          current_position.y() + max_distance,
                          current_position.z() + max_distance);
      iterator = octree->begin_leafs_bbx(minPt, maxPt);
      for (; iterator != octree->end_leafs_bbx(); ++iterator) {
          octomap::point3d coord = iterator.getCoordinate();
          if (!octree->isNodeOccupied(*iterator)) {
              if (isFrontierPoint(coord)) {
                  geometry_msgs::Point frontier_point;
                  frontier_point.x = coord.x();
                  frontier_point.y = coord.y();
                  frontier_point.z = coord.z();
                  frontier_points.push_back(frontier_point);
              }
          }
      }

      if (frontier_points.empty()) {
        ROS_WARN("Error: frontier_points is empty!");
      }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr generateFrontierCloud() {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
      cloud->header.frame_id = "world";
      cloud->is_dense = false;

      for (const auto &point : frontier_points) {
          cloud->points.emplace_back(point.x, point.y, point.z);
      }
      cloud->width = cloud->points.size();
      cloud->height = 1;

      return cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr identifyLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int max_size = 0;
    pcl::PointIndices::Ptr largest_cluster_indices;
    for (const auto &indices : cluster_indices) {
      if (indices->indices.size() > max_size) {
        largest_cluster_indices = indices;
        max_size = indices->indices.size();
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    if (largest_cluster_indices) {
      for (int index : largest_cluster_indices->indices) {
        largest_cluster->points.push_back(cloud->points[index]);
      }
    }
    return largest_cluster;
  }

  pcl::PointXYZ calculateGoal(pcl::PointCloud<pcl::PointXYZ> &cloud) {
    pcl::PointXYZ goal(0, 0, 0);
    for (const auto &point : cloud.points) {
      goal.x += point.x;
      goal.y += point.y;
      goal.z += point.z;
    }
    goal.x /= cloud.points.size();
    goal.y /= cloud.points.size();
    goal.z /= cloud.points.size();
    return goal;
  }

  void publishGoal(const pcl::PointXYZ &goal) {
    geometry_msgs::Point goal_msg;
    goal_msg.x = goal.x;
    goal_msg.y = goal.y;
    goal_msg.z = goal.z;
    if(goal_msg.x <= -340.0){ // 如果无人机位于洞口附近，则发送frontier_goal
      goal_publisher.publish(goal_msg);
    }
  }

  bool isFrontierPoint(const octomap::point3d &coord) {
    // 定义3D邻域搜索偏移量，包括对角线方向
    std::vector<octomap::point3d> searchOffsets =
      {{1, 0, 0},   {-1, 0, 0},   {0, 1, 0},
       {0, -1, 0},  {0, 0, 1},    {0, 0, -1},
       {1, 1, 0},   {-1, -1, 0},  {1, -1, 0},
       {-1, 1, 0}, // XY平面对角线
       {1, 0, 1},   {-1, 0, -1},  {0, 1, 1},
       {0, -1, -1}, // XZ和YZ平面对角线
       {-1, 0, 1},  {1, 0, -1},   {0, -1, 1},
       {0, 1, -1}, // 反对角线
       {1, 1, 1},   {-1, -1, -1}, {1, -1, 1},
       {-1, 1, -1}, // 3D对角线
       {1, 1, -1},  {-1, -1, 1},  {1, -1, -1},
       {-1, 1, 1}};

    for (const auto &offset : searchOffsets) {
      octomap::point3d neighborCoord = coord + offset * octomap_resolution; // 根据octomap分辨率调整邻域偏移
      octomap::OcTreeNode *node = octree->search(neighborCoord);
      if (!node) { // 若节点不存在（未知区域）
        return true;
      }
    }
    return false; // 若所有邻域节点均已知（占用或空闲），则不是frontier
  }

  /* 新增：使用PCL库实现DBSCAN聚类算法 */
  std::vector<pcl::PointIndicesPtr> dbscanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps, int minPts) {
    const int UNCLASSIFIED = -1;
    const int NOISE = -2;
    int n_points = cloud->points.size();
    std::vector<int> labels(n_points, UNCLASSIFIED);

    if (cloud->empty()) {
      PCL_ERROR("Input cloud is empty!\n");
      return {};
    }

    // 使用PCL的KdTree进行邻域搜索
    typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    // lambda函数实现邻域查询
    auto regionQuery = [&](int pointIdx) -> std::vector<int> {
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      tree->radiusSearch(cloud->points[pointIdx], eps, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      return pointIdxRadiusSearch;
    };

    // lambda函数扩展聚类
    std::function<bool(int, int)> expandCluster = [&](int pointIdx, int clusterId) -> bool {
      std::vector<int> seeds = regionQuery(pointIdx);
      if(seeds.size() < static_cast<size_t>(minPts)) {
        labels[pointIdx] = NOISE;
        return false;
      } else {
        for (int seedIdx : seeds) {
          labels[seedIdx] = clusterId;
        }
        // 从种子中去除当前点
        seeds.erase(std::remove(seeds.begin(), seeds.end(), pointIdx), seeds.end());
        // 扩展聚类
        while (!seeds.empty()) {
          int currentP = seeds.back();
          seeds.pop_back();
          std::vector<int> result = regionQuery(currentP);
          if(result.size() >= static_cast<size_t>(minPts)) {
            for (int resultP : result) {
              if (labels[resultP] == UNCLASSIFIED || labels[resultP] == NOISE) {
                if(labels[resultP] == UNCLASSIFIED) {
                  seeds.push_back(resultP);
                }
                labels[resultP] = clusterId;
              }
            }
          }
        }
        return true;
      }
    };

    int clusterId = 0;
    for (int i = 0; i < n_points; ++i) {
      if (labels[i] == UNCLASSIFIED) {
        if(expandCluster(i, clusterId)) {
          clusterId++;
        }
      }
    }

    // 按聚类id将点分组
    std::vector<pcl::PointIndicesPtr> clusters;
    clusters.resize(clusterId);
    for (int i = 0; i < clusterId; ++i) {
      clusters[i] = pcl::PointIndicesPtr(new pcl::PointIndices);
    }
    for (int i = 0; i < n_points; ++i) {
      int label = labels[i];
      if(label >= 0) {
        clusters[label]->indices.push_back(i);
      }
    }
    return clusters;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "FrontierEstimation_node");
  FrontierEstimationNode fe;
  ros::spin();
  return 0;
}
