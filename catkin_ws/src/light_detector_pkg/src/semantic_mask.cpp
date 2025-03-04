#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

class SemanticMaskNode {
public:
  SemanticMaskNode()
  {
    // Subscribers
    semantic_image_sub_ = nh_.subscribe("/realsense/semantic/image_raw", 5,
                                        &SemanticMaskNode::onSemanticImageReceived, this);
    depth_image_sub_    = nh_.subscribe("/realsense/depth/image", 5,
                                        &SemanticMaskNode::onDepthImageReceived, this);
    depth_info_sub_     = nh_.subscribe("/realsense/depth/camera_info", 5,
                                        &SemanticMaskNode::onDepthInfoReceived, this);

    // Publisher for masked depth image
    masked_depth_pub_ = nh_.advertise<sensor_msgs::Image>("masked_depth_image", 5);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber semantic_image_sub_;
  ros::Subscriber depth_image_sub_;
  ros::Subscriber depth_info_sub_;

  ros::Publisher masked_depth_pub_;

  sensor_msgs::Image last_depth_image_;

  // Store depth info if needed
  sensor_msgs::CameraInfo last_depth_info_;
  image_geometry::PinholeCameraModel camera_model_;

private:
  void onSemanticImageReceived(const sensor_msgs::ImageConstPtr &semantic_msg)
  {
    // Only proceed if we also have a valid depth image
    if (last_depth_image_.data.empty()) {
      ROS_WARN_THROTTLE(2.0, "No depth image received yet. Cannot create masked depth.");
      return;
    }

    // Convert semantic image to cv::Mat (BGR8)
    cv_bridge::CvImageConstPtr semantic_cv_ptr;
    try {
      semantic_cv_ptr = cv_bridge::toCvShare(semantic_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Convert the stored depth image to cv::Mat (TYPE_16UC1)
    cv_bridge::CvImagePtr depth_cv_ptr;
    try {
      depth_cv_ptr = cv_bridge::toCvCopy(last_depth_image_, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Create a mask from semantic image (example threshold)
    // Adjust scalar values to match your color mask
    cv::Mat mask;
    cv::inRange(semantic_cv_ptr->image,
                cv::Scalar(4, 235, 255),
                cv::Scalar(4, 235, 255),
                mask);

    // Invalidate depth where mask is false
    // Setting to NaN is one approach; if we want to keep it as zero, adjust accordingly
    depth_cv_ptr->image.setTo(cv::Scalar(std::numeric_limits<uint16_t>::quiet_NaN()), ~mask);

    // Publish the masked depth image
    sensor_msgs::Image masked_depth_msg;
    depth_cv_ptr->toImageMsg(masked_depth_msg);
    masked_depth_msg.header = last_depth_image_.header;
    masked_depth_pub_.publish(masked_depth_msg);
  }

  void onDepthImageReceived(const sensor_msgs::ImageConstPtr &depth_msg)
  {
    last_depth_image_ = *depth_msg;
  }

  void onDepthInfoReceived(const sensor_msgs::CameraInfo &info_msg)
  {
    last_depth_info_ = info_msg;
    camera_model_.fromCameraInfo(info_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "semantic_mask_node");
  SemanticMaskNode node;
  ros::spin();
  return 0;
}
