//Cpp
#include <vector>
#include <math.h>

//ROS
#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <freefloating_gazebo/ControlType.h>
#include <freefloating_gazebo/hydro_model_parser.h>

double clamp(double x, double xmin, double xmax)
{
  return std::min(xmax, std::max(x, xmin));
}

typedef sensor_msgs::Joy::_axes_type Axes;
typedef sensor_msgs::Joy::_buttons_type Buttons;
typedef enum
{
  POSITION_MODE,
  VELOCITY_MODE,
  EFFORT_MODE,
  DEPTH_MODE
} ControlMode;

class StateListener
{
public:
  StateListener(ros::NodeHandle &nh)
  {
    state_sub = nh.subscribe<nav_msgs::Odometry>("state", 10, &StateListener::stateCB, this);
  }
  void stateCB(const nav_msgs::OdometryConstPtr &msg)
  {
    state = msg->pose.pose;
  }
  bool stateReceived()
  {
    return std::abs(state.orientation.x) +
        std::abs(state.orientation.y) +
        std::abs(state.orientation.z) +
        std::abs(state.orientation.w) > 1e-3;
  }
  double yaw()
  {
    tf::Quaternion q;
    // get RPY from quaternion
    tf::quaternionMsgToTF(state.orientation, q);
    double roll, pitch, y;
    tf::Matrix3x3(q).getRPY(roll, pitch, y);
    return y;
  }
  geometry_msgs::Pose state;
  ros::Subscriber state_sub;
};

class JoyListener
{
public:
  JoyListener(ros::NodeHandle &nh)
  {
    joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &JoyListener::joyCB, this);
    cmd.clear();
    cmd["x"] = 0;
    cmd["y"] = 0;
    cmd["z"] = 0;
    cmd["yaw"] = 0;
    cmd["tilt"] = 0;
  }

  void joyCB(const sensor_msgs::Joy::ConstPtr &msg)
  {
    cmd["x"] = static_cast<double>(msg->axes[1]);
    cmd["y"] = static_cast<double>(msg->axes[0]);
    cmd["z"] = static_cast<double>(msg->axes[4]);
    cmd["yaw"] = static_cast<double>(msg->axes[3]);
    cmd["tilt"] = static_cast<double>(-msg->axes[7]);
    buttons = msg->buttons;

  }
  std::map<std::string, double> cmd;
  ros::Subscriber joy_sub;
  Buttons buttons;
};


class Publisher
{
public:
  Publisher(ros::NodeHandle &nh)
  {
    // Service Subscription
    position_client = nh.serviceClient<freefloating_gazebo::ControlType>("controllers/body_position_control");
    velocity_client = nh.serviceClient<freefloating_gazebo::ControlType>("controllers/body_velocity_control");
    effort_client = nh.serviceClient<freefloating_gazebo::ControlType>("controllers/body_effort_control");
  }

  void toPositionMode()
  {
    if(mode != POSITION_MODE)
    {
      srv.request.axes = {"x", "y", "z", "roll", "pitch", "yaw"};
      position_client.call(srv);
      mode = POSITION_MODE;
    }
  }
  void toVelocityMode()
  {
    if(mode != VELOCITY_MODE)
    {
      srv.request.axes = {"x", "y", "z", "roll", "pitch", "yaw"};
      velocity_client.call(srv);
      mode = VELOCITY_MODE;
    }
  }
  void toEffortMode()
  {
    if(mode != EFFORT_MODE)
    {
      srv.request.axes = {"x", "y", "z", "roll", "pitch", "yaw"};
      effort_client.call(srv);
      mode = EFFORT_MODE;
    }
  }
  void toDepthMode()
  {
    if(mode != DEPTH_MODE)
    {
      srv.request.axes = {"z", "roll", "pitch"};
      position_client.call(srv);
      srv.request.axes = {"x", "y", "yaw"};
      velocity_client.call(srv);
      mode = DEPTH_MODE;
    }
  }

  // Service Subscription
  ros::ServiceClient position_client, velocity_client, effort_client;
  ControlMode mode = VELOCITY_MODE;
  freefloating_gazebo::ControlType srv;
};

void update(double &x, double axis, double incr)
{
  x += axis * incr;
}
void update(double &x, double axis, double incr, double xmin, double xmax)
{
  x = clamp(x + axis * incr, xmin, xmax);
}
void quaternionFromYaw(double y, geometry_msgs::Quaternion &q)
{
  q.w = cos(y/2);
  q.z = sin(y/2);
}

std::tuple<std::vector<double>, std::vector<double>> getModelInfo(ros::NodeHandle &nh)
{
  ffg::HydroModelParser parser;
  parser.parseAll(nh, false);
  return {parser.maxVelocity(), parser.maxWrench()};
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "joy_control_node");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  JoyListener joy(nh);
  StateListener state_listener(nh);
  Publisher publisher(nh);

  ros::Publisher position_pub = nh.advertise<geometry_msgs::PoseStamped>("body_position_setpoint",1);
  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("body_velocity_setpoint",1);
  ros::Publisher wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("body_wrench_setpoint",1);
  ros::Publisher tilt_pub = nh.advertise<sensor_msgs::JointState>("joint_setpoint",1);

  const auto [max_vel, max_wrench] = getModelInfo(nh);

  // wait for first messages to come in
  while(!state_listener.stateReceived() || joy.buttons.size() == 0)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // init setpoints, position is initialized to current one without roll pitch
  double yaw = state_listener.yaw();
  geometry_msgs::PoseStamped position_msg;
  position_msg.pose.position = state_listener.state.position;
  quaternionFromYaw(yaw, position_msg.pose.orientation);
  geometry_msgs::TwistStamped twist_msg;
  geometry_msgs::WrenchStamped wrench_msg;
  sensor_msgs::JointState tilt_msg;
  tilt_msg.name = {"tilt"};
  tilt_msg.position = {0};
  position_msg.header.frame_id = "world";
  twist_msg.header.frame_id = "base_link";
  wrench_msg.header.frame_id = "base_link";

  // increments
  double del_incr = 0.05;
  double del_incr_pos = 0.1;
  // sat
  const double tilt_sat = M_PI/4;

  while (ros::ok())
  {
    const auto t = ros::Time::now();

    // update control mode if needed
    if(joy.buttons[2])
      publisher.toPositionMode();
    else if(joy.buttons[0])
      publisher.toDepthMode();
    else if(joy.buttons[3])
      publisher.toEffortMode();
    else if(joy.buttons[1])
      publisher.toVelocityMode();

    // save current pose as desired
    if(joy.buttons[8])
    {
      yaw = state_listener.yaw();
      position_msg.pose.position = state_listener.state.position;
      quaternionFromYaw(yaw, position_msg.pose.orientation);
      std::cout << "Desired pose saved at (";
      std::cout << position_msg.pose.position.x << ", ";
      std::cout << position_msg.pose.position.y << ", ";
      std::cout << position_msg.pose.position.z << ", ";
      std::cout << yaw << ")" << std::endl;
    }

    // tilt angle
    if(joy.buttons[6])
      tilt_msg.position[0] = 0;
    else
      update(tilt_msg.position[0], joy.cmd["tilt"], del_incr, -tilt_sat, tilt_sat);
    tilt_msg.header.stamp = t;
    tilt_pub.publish(tilt_msg);

    switch (publisher.mode)
    {
    case POSITION_MODE:
      update(position_msg.pose.position.x, joy.cmd["x"], del_incr_pos);
      update(position_msg.pose.position.y, joy.cmd["y"], del_incr_pos);
      update(position_msg.pose.position.z, joy.cmd["z"], del_incr_pos, -100, 0);
      update(yaw, joy.cmd["yaw"], del_incr);
      quaternionFromYaw(yaw, position_msg.pose.orientation);

      position_msg.header.stamp = t;
      position_pub.publish(position_msg);
      break;

    case VELOCITY_MODE:
      twist_msg.twist.linear.x = joy.cmd["x"] * max_vel[0];
      twist_msg.twist.linear.y = joy.cmd["y"] * max_vel[1];
      twist_msg.twist.linear.z = joy.cmd["z"] * max_vel[2];
      twist_msg.twist.angular.z = joy.cmd["yaw"] * max_vel[5];

      twist_msg.header.stamp = t;
      velocity_pub.publish(twist_msg);
      break;

    case EFFORT_MODE:
      wrench_msg.wrench.force.x = joy.cmd["x"] * max_wrench[0];
      wrench_msg.wrench.force.y = joy.cmd["y"] * max_wrench[1];
      wrench_msg.wrench.force.z = joy.cmd["z"] * max_wrench[2];
      wrench_msg.wrench.torque.z = joy.cmd["yaw"] * max_wrench[5];

      wrench_msg.header.stamp = t;
      wrench_pub.publish(wrench_msg);
      break;

    case DEPTH_MODE:
      twist_msg.twist.linear.x = joy.cmd["x"] * max_vel[0];
      twist_msg.twist.linear.y = joy.cmd["y"] * max_vel[1];
      twist_msg.twist.angular.z = joy.cmd["yaw"] * max_vel[5];
      update(position_msg.pose.position.z, joy.cmd["z"], del_incr_pos, -100, 0);

      position_msg.header.stamp = t;
      position_pub.publish(position_msg);
      twist_msg.header.stamp = t;
      velocity_pub.publish(twist_msg);
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }

}
