#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdlib>
#include <iostream>



class StateMachine
{
public:
    StateMachine();

private:
    enum class StatusPhase
    {
        ascend_step,
        transit_cave,
        stationary,
        descent,
        rotate_heading,
        move_straight
    };

    geometry_msgs::Point fetchNextGoalPoint();
    void receivePathData(const nav_msgs::Path::ConstPtr &msg);
    geometry_msgs::PoseStamped convertPointToPoseMsg(const geometry_msgs::Point &point,
                                                     const std::string &frame_id,
                                                     double yaw = 0.0);
    void insertTargetPoint(double x, double y, double z);
    void executeMissionSequence(const ros::TimerEvent &t);

    // Individual state handlers
    void performAscent();
    void moveIntoCave();
    void sustainHover();
    void initiateDescent();
    void pivotDrone();
    void advanceDrone();

    // Callbacks, checks, utility
    void obtainCurrentOdom(const nav_msgs::Odometry &cur_state);
    bool withinThreshold(double low, double high, double x);
    bool targetFulfilled();
    void configureWaypoint(tf::Vector3 pos, tf::Quaternion q,
                           tf::Vector3 lin_vel = tf::Vector3(0, 0, 0),
                           tf::Vector3 ang_vel = tf::Vector3(0, 0, 0),
                           tf::Vector3 lin_acc = tf::Vector3(0, 0, 0));
    void storePosition();

    // ROS-related members
    ros::NodeHandle nh;
    ros::Publisher desired_state_pub_;
    ros::Publisher goal_position_pub_;
    ros::Subscriber current_state_sub_;
    ros::Subscriber path_sub_;
    ros::Timer coordination_timer_;

    tf::TransformBroadcaster br;

    // Internal data
    std::vector<geometry_msgs::Point> goalpoints;
    std::vector<Eigen::Vector4d> path_waypoints_vec;
    int current_goal_index = 0;

    tf::Quaternion quat_;
    double yaw_ = 0.0;
    double yaw_des = 0.0;
    double tolerance_ = 1.0;

    tf::Vector3 origin_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d omega_;
    Eigen::Vector3d cur_position;

    geometry_msgs::Point goalpoint;
    StatusPhase phase_ = StatusPhase::ascend_step;
    bool waypoint_navigation_launched = false;
    bool goal_dispatched_once = false;

    // Timings, etc.
    double sim_interval_ = 0.1;

    // Utility
    double computeYawForWaypoint(const geometry_msgs::Point &current,
                                 const geometry_msgs::Point &next)
    {
        double delta_x = next.x - current.x;
        double delta_y = next.y - current.y;
        return std::atan2(delta_y, delta_x);
    }
};

StateMachine::StateMachine()
    : waypoint_navigation_launched(false)
{
    double x, y, z;
    x = -38.0;
    y = 10.0;
    z = 6.9;

    // Insert same goal points but keep function name changed
    insertTargetPoint(-38.0, 10.0, 10.0); // take off from initial position
    insertTargetPoint(-55, 0.84, 15.0);   // first lamp position at the outside
    insertTargetPoint(-321, 10.0, 15.0);  // cave entrance
    insertTargetPoint(-500, 0.0, 10.0);
    insertTargetPoint(-599, -9.0, 10.0);
    insertTargetPoint(-599, -5, 10.0);
    insertTargetPoint(-599, -2, 3.0);

    origin_ = tf::Vector3(x, y, z);

    desired_state_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "command/trajectory", 1);
    current_state_sub_ = nh.subscribe("current_state_est", 1,
                                      &StateMachine::obtainCurrentOdom,
                                      this);
    goal_position_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "goal_position", 1);
    path_sub_ = nh.subscribe("planned_path", 1,
                             &StateMachine::receivePathData,
                             this);

    coordination_timer_ = nh.createTimer(ros::Duration(sim_interval_),
                                         &StateMachine::executeMissionSequence,
                                         this);
}

geometry_msgs::Point StateMachine::fetchNextGoalPoint()
{
    if (current_goal_index < (int)goalpoints.size())
    {
        return goalpoints[current_goal_index++];
    }
    else
    {
        // Return the last point or handle the end as needed
        return goalpoints.back();
    }
}

void StateMachine::receivePathData(const nav_msgs::Path::ConstPtr &msg)
{
    path_waypoints_vec.clear();
    auto received_poses = msg->poses;

    // Ensure the path has at least three points by adding a midpoint if needed
    if (received_poses.size() == 2)
    {
        Eigen::Vector3d first_point(
            received_poses[0].pose.position.x,
            received_poses[0].pose.position.y,
            received_poses[0].pose.position.z);
        Eigen::Vector3d second_point(
            received_poses[1].pose.position.x,
            received_poses[1].pose.position.y,
            received_poses[1].pose.position.z);
        Eigen::Vector3d mid_point = (first_point + second_point) * 0.5;

        geometry_msgs::PoseStamped new_pose;
        new_pose.pose.position.x = mid_point(0);
        new_pose.pose.position.y = mid_point(1);
        new_pose.pose.position.z = mid_point(2);
        received_poses.insert(received_poses.begin() + 1, new_pose);
    }

    // Compute orientation data (yaw) for each path point
    for (size_t i = 0; i < received_poses.size(); ++i)
    {
        auto current_pose = received_poses[i];
        double posX = current_pose.pose.position.x;
        double posY = current_pose.pose.position.y;
        double posZ = current_pose.pose.position.z;
        double orientation_yaw;
        Eigen::Vector3d current_position(posX, posY, posZ), next_position;
        size_t idx_next = i + 1;

        if (idx_next < received_poses.size())
        {
            double nextPosX = received_poses[idx_next].pose.position.x;
            double nextPosY = received_poses[idx_next].pose.position.y;
            double nextPosZ = received_poses[idx_next].pose.position.z;
            next_position = Eigen::Vector3d(nextPosX, nextPosY, nextPosZ);

            Eigen::Vector3d direction_vector = next_position - current_position;
            orientation_yaw = std::atan2(direction_vector[1], direction_vector[0]);
        }
        else
        {
            // Reuse previous yaw for the last point
            orientation_yaw = path_waypoints_vec[i - 1][3];
        }

        path_waypoints_vec.push_back(Eigen::Vector4d(posX, posY, posZ, orientation_yaw));
    }

    // Transform the last waypoint to a PoseStamped
    if (!path_waypoints_vec.empty())
    {
        const Eigen::Vector4d &goalPoint = path_waypoints_vec.back();
        geometry_msgs::PoseStamped goal_pose_stamped;
        goal_pose_stamped.pose.position.x = goalPoint[0];
        goal_pose_stamped.pose.position.y = goalPoint[1];
        goal_pose_stamped.pose.position.z = goalPoint[2];

        tf2::Quaternion q;
        q.setRPY(0, 0, goalPoint[3]);

        goal_pose_stamped.pose.orientation = tf2::toMsg(q);

        goal_pose_stamped.header.frame_id = "world";
        goal_pose_stamped.header.stamp = ros::Time::now();

        goal_position_pub_.publish(goal_pose_stamped);
    }
}

geometry_msgs::PoseStamped StateMachine::convertPointToPoseMsg(
    const geometry_msgs::Point &point,
    const std::string &frame_id,
    double yaw)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = frame_id;

    pose_stamped.pose.position = point;
    tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
    quaternionTFToMsg(quat, pose_stamped.pose.orientation);

    return pose_stamped;
}

void StateMachine::insertTargetPoint(double x, double y, double z)
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    goalpoints.push_back(point);
}

void StateMachine::executeMissionSequence(const ros::TimerEvent &t)
{
    if (phase_ == StatusPhase::ascend_step)
    {
        performAscent();
    }
    else if (phase_ == StatusPhase::transit_cave)
    {
        moveIntoCave();
    }
    else if (phase_ == StatusPhase::stationary)
    {
        sustainHover();
    }
    else if (phase_ == StatusPhase::descent)
    {
        initiateDescent();
    }
    else if (phase_ == StatusPhase::rotate_heading)
    {
        pivotDrone();
    }
    else if (phase_ == StatusPhase::move_straight)
    {
        advanceDrone();
    }
}

void StateMachine::performAscent()
{
    ROS_INFO_ONCE("Drone is taking off!");
    if (!goal_dispatched_once)
    {
        goal_dispatched_once = true;
        goalpoint = fetchNextGoalPoint();
        ros::Duration(1).sleep();
        auto goal_pose_stamped = convertPointToPoseMsg(goalpoint, "world");
        goal_position_pub_.publish(goal_pose_stamped);
        ROS_INFO("Published goal position: [%f, %f, %f]",
                 goalpoint.x, goalpoint.y, goalpoint.z);
    }

    if (targetFulfilled())
    {
        goal_dispatched_once = false;
        phase_ = StatusPhase::transit_cave;
    }
}

void StateMachine::moveIntoCave()
{
    ROS_INFO_ONCE("Drone is flying to cave!");
    if (!goal_dispatched_once)
    {
        goal_dispatched_once = true;
        goalpoint = fetchNextGoalPoint();
        auto goal_pose_stamped = convertPointToPoseMsg(goalpoint, "world", -1.5708 * 2);

        goal_position_pub_.publish(goal_pose_stamped);
        ROS_INFO("Published goal position: [%f, %f, %f]",
                 goalpoint.x, goalpoint.y, goalpoint.z);
    }

    if (targetFulfilled())
    {
        goal_dispatched_once = false;
        if (current_goal_index == 3)
        {
            phase_ = StatusPhase::stationary;
            yaw_des = -1.5708;
            storePosition();
        }
    }
}

void StateMachine::sustainHover()
{
    ROS_INFO_ONCE("Drone is hovering!");
}

void StateMachine::initiateDescent()
{
    ROS_INFO_ONCE("Drone is landing!");
    tf::Vector3 pos(cur_position[0], cur_position[1], 0);
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    configureWaypoint(pos, q);
}

void StateMachine::pivotDrone()
{
    ROS_INFO_ONCE("Drone is turning!");
    tf::Vector3 pos(cur_position[0], cur_position[1], cur_position[2]);
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_des);

    configureWaypoint(pos, q);

    if (withinThreshold(yaw_des - 0.01, yaw_des + 0.01, yaw_))
    {
        phase_ = StatusPhase::move_straight;
        storePosition();
    }
}

void StateMachine::advanceDrone()
{
    ROS_INFO_ONCE("Drone is flying forward!");
    tf::Vector3 pos(cur_position[0], cur_position[1] - 3, cur_position[2]);
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_des);
    configureWaypoint(pos, q);

    if (withinThreshold(pos_[1] - tolerance_, pos_[1] + tolerance_,
                        cur_position[1] - 5))
    {
        phase_ = StatusPhase::stationary;
        storePosition();
    }
}

void StateMachine::obtainCurrentOdom(const nav_msgs::Odometry &cur_state)
{
    pos_ << cur_state.pose.pose.position.x,
        cur_state.pose.pose.position.y,
        cur_state.pose.pose.position.z;
    vel_ << cur_state.twist.twist.linear.x,
        cur_state.twist.twist.linear.y,
        cur_state.twist.twist.linear.z;
    omega_ << cur_state.twist.twist.angular.x,
        cur_state.twist.twist.angular.y,
        cur_state.twist.twist.angular.z;

    tf::quaternionMsgToTF(cur_state.pose.pose.orientation, quat_);
    yaw_ = tf::getYaw(quat_);
}

bool StateMachine::withinThreshold(double low, double high, double x)
{
    return ((x - high) * (x - low) <= 0);
}

bool StateMachine::targetFulfilled()
{
    return (withinThreshold(goalpoint.x - tolerance_, goalpoint.x + tolerance_, pos_[0]) &&
            withinThreshold(goalpoint.y - tolerance_, goalpoint.y + tolerance_, pos_[1]) &&
            withinThreshold(goalpoint.z - tolerance_, goalpoint.z + tolerance_, pos_[2]));
}

void StateMachine::configureWaypoint(tf::Vector3 pos, tf::Quaternion q,
                                           tf::Vector3 lin_vel,
                                           tf::Vector3 ang_vel,
                                           tf::Vector3 lin_acc)
{
    tf::Transform desired_pos(tf::Transform::getIdentity());
    geometry_msgs::Twist vel;
    geometry_msgs::Twist acc;

    desired_pos.setOrigin(pos);
    desired_pos.setRotation(q);

    vel.linear.x = lin_vel.getX();
    vel.linear.y = lin_vel.getY();
    vel.linear.z = lin_vel.getZ();
    vel.angular.x = ang_vel.getX();
    vel.angular.y = ang_vel.getY();
    vel.angular.z = ang_vel.getZ();

    acc.linear.x = lin_acc.getX();
    acc.linear.y = lin_acc.getY();
    acc.linear.z = lin_acc.getZ();

    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.transforms[0].translation.x = desired_pos.getOrigin().x();
    msg.transforms[0].translation.y = desired_pos.getOrigin().y();
    msg.transforms[0].translation.z = desired_pos.getOrigin().z();
    msg.transforms[0].rotation.x = desired_pos.getRotation().getX();
    msg.transforms[0].rotation.y = desired_pos.getRotation().getY();
    msg.transforms[0].rotation.z = desired_pos.getRotation().getZ();
    msg.transforms[0].rotation.w = desired_pos.getRotation().getW();

    msg.velocities.resize(1);
    msg.velocities[0] = vel;
    msg.accelerations.resize(1);
    msg.accelerations[0] = acc;

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.points.push_back(msg);
    desired_state_pub_.publish(trajectory_msg);

    br.sendTransform(tf::StampedTransform(desired_pos, ros::Time::now(),
                                          "world", "av-desired"));
}

void StateMachine::storePosition()
{
    cur_position << pos_[0], pos_[1], pos_[2];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "machine_state");
    ROS_INFO_ONCE("StateMachine node initialized.");
    StateMachine StateMachine1;
    ros::spin();
    return 0;
}
