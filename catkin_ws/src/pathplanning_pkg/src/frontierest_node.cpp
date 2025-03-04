#include "Optics.hpp"
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
#include <ros/ros.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <vector>

class FrontierEstimationNode {
public:
  FrontierEstimationNode() {
    state_subscriber = node_handle.subscribe(
        "current_state_est", 1, &FrontierEstimationNode::currentStateCallback, this);
    map_subscriber = node_handle.subscribe("octomap_full", 1,
                                         &FrontierEstimationNode::mapUpdateCallback, this);
    goal_publisher = node_handle.advertise<geometry_msgs::Point>("frontier_goal", 1);

    node_handle.getParam("/octomap_server/resolution", octomap_resolution);
    exploration_timer = node_handle.createTimer(ros::Rate(0.3), &FrontierEstimationNode::explorationTimerCallback, this);
  }

private:
  ros::NodeHandle node_handle;
  ros::Subscriber state_subscriber, map_subscriber;
  ros::Publisher goal_publisher;
  ros::Timer exploration_timer;

  octomap::OcTree::leaf_bbx_iterator iterator;
  std::shared_ptr<octomap::OcTree> octree{};
  octomap::point3d current_position;
  std::vector<geometry_msgs::Point> frontier_points;
  int octomap_resolution, max_distance; 
  std::vector<pcl::PointIndicesPtr> cluster_indices;

  void currentStateCallback(const nav_msgs::Odometry &msg) {
    current_position =
        octomap::point3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
  }

  void mapUpdateCallback(const octomap_msgs::OctomapConstPtr &msg) {
    octree = std::make_shared<octomap::OcTree>(*dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
  }

  void explorationTimerCallback(const ros::TimerEvent &event) {
    if (!octree)
      return;

    detectFrontiers();
    auto frontier_cloud = generateFrontierCloud();

    if (!frontier_cloud) {
      std::cerr << "[ERROR] cloud is a null pointer!" << std::endl;
    } else if (frontier_cloud->empty()) {
      std::cerr << "[ERROR] cloud is empty!" << std::endl;
    } else {
      std::cout << "[INFO] cloud contains " << frontier_cloud->size() << " points." << std::endl;
    }

   
    Optics::optics<pcl::PointXYZ>(frontier_cloud, 5, 10.0, cluster_indices);

    auto largest_cluster_cloud = identifyLargestCluster(frontier_cloud);
    //auto largest_cluster_cloud = clusterDBSCAN(frontier_cloud, 0.03, 30);
    auto goal = calculateGoal(*largest_cluster_cloud);

    ROS_INFO("Frontier goal point set to: (%f, %f, %f)", goal.x, goal.y, goal.z);
    publishGoal(goal);
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
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr clusterDBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr frontier_cloud, float eps, int minPts) {
    if (!frontier_cloud || frontier_cloud->empty()) {
      std::cerr << "[ERROR] Input cloud is empty!" << std::endl;
      return nullptr;
    }

    // KdTree for nearest neighbor search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(frontier_cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(eps);  // 设定DBSCAN的epsilon参数
    ec.setMinClusterSize(minPts); // 设定最小点数
    ec.setMaxClusterSize(frontier_cloud->size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(frontier_cloud);
    ec.extract(cluster_indices);
    
    if (cluster_indices.empty()) {
        return nullptr; // 没有找到任何簇
    }
    
    // 找到最大的簇
    auto largest_cluster = std::max_element(
        cluster_indices.begin(), cluster_indices.end(),
        [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() < b.indices.size();
        });
    
    // 提取最大簇
    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr largest_cluster_indices(new pcl::PointIndices(*largest_cluster));
    extract.setInputCloud(frontier_cloud);
    extract.setIndices(largest_cluster_indices);
    extract.setNegative(false);
    extract.filter(*largest_cluster_cloud);
    
    return largest_cluster_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr identifyLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int max_size = 0;
    pcl::PointIndices::Ptr largest_cluster_indices;
    for (const auto &cluster_indices : cluster_indices) {
      if (cluster_indices->indices.size() > max_size) {
        largest_cluster_indices = cluster_indices;
        max_size = cluster_indices->indices.size();
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
    if (cloud.points.empty())
      return goal;

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
  
  void detectFrontiers() {
    frontier_points.clear();
    octomap::point3d minPt(current_position.x() - max_distance,
                         current_position.y() - max_distance,
                         current_position.z() - max_distance);
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
  }
 
  bool isFrontierPoint(const octomap::point3d &coord) {
    std::vector<octomap::point3d> searchOffsets =
        {{1, 0, 0},   {-1, 0, 0},   {0, 1, 0},
         {0, -1, 0},  {0, 0, 1},    {0, 0, -1},
         {1, 1, 0},   {-1, -1, 0},  {1, -1, 0},
         {-1, 1, 0}, 
         {1, 0, 1},   {-1, 0, -1},  {0, 1, 1},
         {0, -1, -1}, 
         {-1, 0, 1},  {1, 0, -1},   {0, -1, 1},
         {0, 1, -1}, 
         {1, 1, 1},   {-1, -1, -1}, {1, -1, 1},
         {-1, 1, -1}, 
         {1, 1, -1},  {-1, -1, 1},  {1, -1, -1},
         {-1, 1, 1}};

    for (const auto &offset : searchOffsets) {
      octomap::point3d neighborCoord =
          coord + offset * octomap_resolution; 
      octomap::OcTreeNode *node = octree->search(neighborCoord);
      if (!node) { 
        return true;
      }
    }
    return false; 
  }
  
  void publishGoal(const pcl::PointXYZ &goal) {
    geometry_msgs::Point goal_msg;
    goal_msg.x = goal.x;
    goal_msg.y = goal.y;
    goal_msg.z = goal.z;
    if(goal_msg.x<=-340.0){goal_publisher.publish(goal_msg);}
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "FrontierEstimation_node");
  FrontierEstimationNode fe;

  ros::spin();
  return 0;
}
