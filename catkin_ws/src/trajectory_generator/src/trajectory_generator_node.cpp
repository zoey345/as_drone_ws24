#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <trajectory_generator.h>


class BasicPlanner {

// private:
//     ros::Publisher pub_markers_;
//     ros::Publisher pub_trajectory_;
//     ros::Subscriber sub_odom_;

//     ros::NodeHandle& nh_;
//     Eigen::Vector4d current_pose_;
//     Eigen::Vector4d current_velocity_;
//     Eigen::Vector4d current_angular_velocity_;
//     double max_v_; // m/s
//     double max_a_; // m/s^2
//     double max_ang_v_;
//     double max_ang_a_;


public:
    BasicPlanner(ros::NodeHandle &nh) : nh_(nh),
                                        max_v_(30.0),
                                        max_a_(10.0),
                                        current_velocity_(Eigen::Vector4d::Zero()),
                                        current_pose_(Eigen::Vector4d::Zero()) {
        pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
        pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
        sub_odom_ = nh.subscribe("current_state_est", 1, &BasicPlanner::uavOdomCallback, this);
    }

    void setMaxSpeed(const double max_v) { max_v_ = max_v; }

    bool planTrajectory(const Eigen::VectorXd &goal_pos,
                        const Eigen::VectorXd &goal_vel,
                        mav_trajectory_generation::Trajectory *trajectory) {
        const int dimension = 4;
        mav_trajectory_generation::Vertex::Vector vertices;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

        mav_trajectory_generation::Vertex start(dimension), end(dimension);
        start.makeStartOrEnd(current_pose_, derivative_to_optimize);
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_velocity_);
        vertices.push_back(start);

        end.makeStartOrEnd(goal_pos, derivative_to_optimize);
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_vel);
        vertices.push_back(end);

        std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

        const int N = 10;
        //mav_trajectory_generation::NonlinearOptimizationParameters parameters;
        mav_trajectory_generation::NonlinearOptimizationParameters parameters;
        //mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
        mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
        opt.optimize();
        opt.getTrajectory(&(*trajectory));

        return true;
    }

    bool publishTrajectory(const mav_trajectory_generation::Trajectory &trajectory) {
        visualization_msgs::MarkerArray markers;
        double distance = 0.2;
        std::string frame_id = "world";
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        pub_markers_.publish(markers);

        mav_planning_msgs::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
        msg.header.frame_id = "world";
        pub_trajectory_.publish(msg);

        return true;
    }

//private:
    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
        Eigen::Quaterniond qd(odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y,
                              odom->pose.pose.orientation.z,
                              odom->pose.pose.orientation.w);
        current_pose_ << odom->pose.pose.position.x,
                        odom->pose.pose.position.y,
                        odom->pose.pose.position.z,
                        mav_msgs::yawFromQuaternion(qd);
        current_velocity_ << odom->twist.twist.linear.x,
                             odom->twist.twist.linear.y,
                             odom->twist.twist.linear.z,
                             0.0;
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_markers_, pub_trajectory_;
    ros::Subscriber sub_odom_;
    double max_v_, max_a_;
    Eigen::Vector4d current_velocity_, current_pose_;
};

// Global variables for callback handling
BasicPlanner* planner = nullptr;
Eigen::Vector4d last_goal_pose = Eigen::Vector4d::Zero();

bool isDifferentPose(const Eigen::Vector4d& pose1, const Eigen::Vector4d& pose2) {
    const double threshold = 0.01;
    return (pose1.head<3>() - pose2.head<3>()).norm() > threshold;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double yaw = tf::getYaw(msg->pose.orientation);
    Eigen::Vector4d goal_pose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, yaw);
    Eigen::Vector4d goal_velocity(0.0, 0.0, 0.0, 0.0);

    if (isDifferentPose(goal_pose, last_goal_pose)) {
        ROS_INFO_STREAM("Received new goal pose: " << goal_pose.transpose());
        mav_trajectory_generation::Trajectory trajectory;
        planner->planTrajectory(goal_pose, goal_velocity, &trajectory);
        planner->publishTrajectory(trajectory);
        last_goal_pose = goal_pose;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator_node");
    ros::NodeHandle nh;

    planner = new BasicPlanner(nh);
    ros::Subscriber goal_sub = nh.subscribe("goal_position", 1, goalCallback);
    ros::spin();

    delete planner;
    return 0;
}