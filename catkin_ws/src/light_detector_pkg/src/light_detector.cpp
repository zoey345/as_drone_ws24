#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <depth_image_proc/depth_traits.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <vector>

class LightDetectorNode {
public:
  LightDetectorNode()
  {
    // Subscribes to the masked depth image, which was produced by Node 1
    masked_depth_sub_ = nh_.subscribe("/masked_depth_image", 5,
                                      &LightDetectorNode::onMaskedDepthImageReceived, this);
    depth_info_sub_    = nh_.subscribe("/realsense/depth/camera_info", 5,
                                       &LightDetectorNode::onDepthInfoReceived, this);

    // Publisher for detected points
    detected_points_pub_ = nh_.advertise<geometry_msgs::PointStamped>("detected_points", 10);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber masked_depth_sub_;
  ros::Subscriber depth_info_sub_;

  ros::Publisher detected_points_pub_;
  tf::TransformListener transform_listener_;

  sensor_msgs::CameraInfo last_depth_info_;
  image_geometry::PinholeCameraModel camera_model_;

  // Keep track of prior detections
  std::vector<pcl::PointXYZ> detections_;

private:
  void onMaskedDepthImageReceived(const sensor_msgs::ImageConstPtr &masked_depth_msg)
  {
    // Convert masked depth to cv::Mat
    cv_bridge::CvImageConstPtr depth_cv_ptr;
    try {
      depth_cv_ptr = cv_bridge::toCvShare(masked_depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Create PointCloud2 from masked depth
    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    cloud_msg->header = masked_depth_msg->header;
    cloud_msg->height = masked_depth_msg->height;
    cloud_msg->width  = masked_depth_msg->width;
    cloud_msg->is_dense     = false;
    cloud_msg->is_bigendian = false;

    // We only store XYZ
    sensor_msgs::PointCloud2Modifier cloud_modifier(*cloud_msg);
    cloud_modifier.setPointCloud2FieldsByString(1, "xyz");

    // Fill in the point cloud from the depth image
    depthImageToPointCloud(depth_cv_ptr->image, cloud_msg, camera_model_);

    // Transform to global (world) frame
    pcl::PointCloud<pcl::PointXYZ> world_frame_cloud = transformPointCloudToGlobalFrame(cloud_msg);
    if (world_frame_cloud.points.empty()) {
      return;
    }

    // Compute center of the valid 3D points
    pcl::PointXYZ detection_center = calculatePointCloudCenter(world_frame_cloud);

    // Check for NaN
    if (!std::isnan(detection_center.x) &&
        !std::isnan(detection_center.y) &&
        !std::isnan(detection_center.z))
    {
      // Check if this is a "new" detection
      if (checkForNewDetection(detection_center)) {
        ROS_WARN("Light detected at %f, %f, %f",
                 detection_center.x,
                 detection_center.y,
                 detection_center.z);

        detections_.push_back(detection_center);

        // Publish detection
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp    = ros::Time::now();
        point_msg.header.frame_id = "world"; // or the desired world frame
        point_msg.point.x = detection_center.x;
        point_msg.point.y = detection_center.y;
        point_msg.point.z = detection_center.z;
        detected_points_pub_.publish(point_msg);
      }
    }
  }

  void onDepthInfoReceived(const sensor_msgs::CameraInfo &info_msg)
  {
    last_depth_info_ = info_msg;
    camera_model_.fromCameraInfo(info_msg);
  }

  void depthImageToPointCloud(const cv::Mat &depth_image,
                              sensor_msgs::PointCloud2::Ptr &cloud_msg,
                              const image_geometry::PinholeCameraModel &cam_model,
                              double range_max = 0.0)
  {
    float center_x = cam_model.cx();
    float center_y = cam_model.cy();

    double unit_scaling = depth_image_proc::DepthTraits<uint16_t>::toMeters(uint16_t(1));
    float constant_x    = unit_scaling / cam_model.fx();
    float constant_y    = unit_scaling / cam_model.fy();
    float bad_point     = std::numeric_limits<float>::quiet_NaN();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    const uint16_t *depth_row = reinterpret_cast<const uint16_t *>(depth_image.data);
    int row_step = depth_image.step / sizeof(uint16_t);

    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step) {
      for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
        uint16_t depth = depth_row[u];
        if (!depth_image_proc::DepthTraits<uint16_t>::valid(depth)) {
          // If invalid and range_max is set, clamp
          if (range_max != 0.0) {
            depth = depth_image_proc::DepthTraits<uint16_t>::fromMeters(range_max);
          } else {
            *iter_x = *iter_y = *iter_z = bad_point;
            continue;
          }
        }

        // Project
        *iter_x = (u - center_x) * depth * constant_x;
        *iter_y = (v - center_y) * depth * constant_y;
        *iter_z = depth_image_proc::DepthTraits<uint16_t>::toMeters(depth);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>
  transformPointCloudToGlobalFrame(sensor_msgs::PointCloud2::Ptr cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ> local_cloud;
    pcl::fromROSMsg(*cloud_msg, local_cloud);

    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    tf::StampedTransform transform;
    try {
      transform_listener_.waitForTransform(
          "world", cloud_msg->header.frame_id,
          ros::Time(0), ros::Duration(10.0));
      transform_listener_.lookupTransform(
          "world", cloud_msg->header.frame_id,
          ros::Time(0), transform);

      pcl_ros::transformPointCloud(local_cloud, global_cloud, transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("Transform error: %s", ex.what());
    }

    return global_cloud;
  }

  pcl::PointXYZ calculatePointCloudCenter(const pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
    pcl::PointXYZ center(0, 0, 0);
    int valid_points = 0;
    for (const auto &pt : cloud.points) {
      if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) &&
          !(pt.x == 0 && pt.y == 0 && pt.z == 0))
      {
        center.x += pt.x;
        center.y += pt.y;
        center.z += pt.z;
        valid_points++;
      }
    }
    if (valid_points > 0) {
      center.x /= valid_points;
      center.y /= valid_points;
      center.z /= valid_points;
    } else {
      center.x = std::numeric_limits<float>::quiet_NaN();
      center.y = std::numeric_limits<float>::quiet_NaN();
      center.z = std::numeric_limits<float>::quiet_NaN();
    }
    return center;
  }

  bool checkForNewDetection(const pcl::PointXYZ &point)
  {
    // Example distance check to avoid duplicates
    for (const auto &existing : detections_) {
      double dist_sq =
          (existing.x - point.x) * (existing.x - point.x) +
          (existing.y - point.y) * (existing.y - point.y) +
          (existing.z - point.z) * (existing.z - point.z);
      if (dist_sq < 100.0) {
        // If it's within 10m (sqrt(100)=10) of an existing detection
        return false;
      }
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "light_detector_node");
  LightDetectorNode node;
  ros::spin();
  return 0;
}
