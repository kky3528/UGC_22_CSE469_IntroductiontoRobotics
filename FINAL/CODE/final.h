#ifndef FINAL_H_
#define FINAL_H_

#include <cstdio>
#include <iostream>
#include <limits>
#include <string>

#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <ros/network.h>
#include <std_msgs/String.h>
#include <sstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <time.h>

// for final
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

using namespace std;

// for terminal
static int terminal_descriptor = -1;
static struct termios terminal_original;
static struct termios terminal_settings;
int terminal_init();
void terminal_done();
void set_terminal_raw_mode();
void set_terminal_original_mode();

// for keyboard input
bool wait_for_key_pressed(unsigned);

// for linear velocity and angular velocity (teleop)
static const float WAFFLE_MAX_LIN_VEL = 0.26;
static const float WAFFLE_MAX_ANG_VEL = 1.82;
static const float LIN_VEL_STEP_SIZE = 0.01;
static const float ANG_VEL_STEP_SIZE = 0.1;
ros::Publisher vel_pub;
void new_velocity(float, float, bool);

// for gripper and arm (manipulation)
const vector<double> gr_open = {0.010};
const vector<double> gr_close = {-0.010};
const vector<double> gr_hold = {-0.002};
const vector<double> gr_finish = {0.000};
const vector<double> ar_home = {0.000, -0.963, 0.314, 0.680};
const vector<double> ar_init = {0.000, 0.031, 0.008, 0.000};
//const vector<double> ar_hold_1 = {0.000, 0.559, -0.933, 0.406};
//const vector<double> ar_hold_2 = {0.000, 1.453, -0.927, -0.548};
const vector<double> ar_hold_1 = {0.000, 1.476, -0.940, -0.196};
const vector<double> ar_hold_2 = {0.000, 1.476, -0.940, -0.836};
moveit::planning_interface::MoveGroupInterface* move_group_;
moveit::planning_interface::MoveGroupInterface* move_group2_;
bool groupinit();
bool position(vector<double>, double);
bool gripper(vector<double>);
void update_cur_joint();
void print_cur_joint();

// for object detection (darknet)
ros::Subscriber object_sub;
static bool detected = false;
void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr&);
void head_object(ros::NodeHandle);
static string object_class ="None";
static int object_xmid = 0;

// for tutlebot status
static int ttb_status = 0;
static float target_linear_vel = 0.0;
static float target_angular_vel = 0.0;
static float control_linear_vel = 0.0;
static float control_angular_vel = 0.0;
static vector<double> arm = ar_home;
static vector<double> hand = gr_finish;
void printText();
static float cur_pos_x = 0.0;
static float cur_pos_y = 0.0;
static float cur_pos_ang = 0.0;
static vector<double> cur_joint = {0.000, 0.000, 0.000, 0.000, 0.000};

// for moving position
ros::Subscriber curpos_sub;
static float goal_pos_x = 0.0;
static float goal_pos_y = 0.0;
const double PI = 3.141592654;
const double K_lin = 0.1;
const double K_ang = 0.15;
const double distance_threshold = 0.2;
const double angle_threshold = 0.15;
static bool moving = false;
void update_cur_pos (const nav_msgs::Odometry::ConstPtr&);
void move_pos(vector<double>, ros::NodeHandle);
const double RT = 10.0;
const vector<double> pos_1 = {0.000*RT, 0.000*RT};
const vector<double> pos_1_edit = {0.000*RT, -0.500*RT};
const vector<double> pos_1_edit2 = {0.1500*RT, -0.600*RT};
const vector<double> pos_2 = {0.742*RT, -0.742*RT};
const vector<double> pos_2_edit = {0.371*RT, -0.371*RT};
const vector<double> pos_3 = {1.484*RT, -1.484*RT};
const vector<double> pos_4 = {0.742*RT, 0.742*RT};
const vector<double> pos_5 = {1.484*RT, 0.000*RT};
const vector<double> pos_6 = {2.226*RT, -0.742*RT};
const vector<double> pos_7 = {1.484*RT, 1.484*RT};
const vector<double> pos_7_edit2 = {1.684*RT, 0.942*RT};
const vector<double> pos_7_edit = {1.484*RT, 0.742*RT};
const vector<double> pos_8 = {2.226*RT, 0.742*RT};
const vector<double> pos_8_edit = {2.226*RT, 0.800*RT};
const vector<double> pos_9 = {2.968*RT, 0.000*RT};

// for object detection and tracking (darknet & laserscan)
ros::Subscriber track_sub;
static bool approach = false;
void head_tracking(ros::NodeHandle);

static bool is_spotted = false;
static int is_spotted_ang = 0;

// for hold and return
void hold_or_return(int);

// delete main

/* Restore terminal to original settings */
void terminal_done() {
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_original);
}

void terminal_signal(int signum) {
  cout << "Terminal_signal(" << signum << ")" << endl;
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_original);
  _exit(128 + signum);
}

int terminal_init() {

  if (terminal_descriptor != -1)
    return errno = 0;

  if (isatty(STDERR_FILENO))
    terminal_descriptor = STDERR_FILENO;
  else
  if (isatty(STDIN_FILENO))
    terminal_descriptor = STDIN_FILENO;
  else
  if (isatty(STDOUT_FILENO))
    terminal_descriptor = STDOUT_FILENO;
  else
    return errno = ENOTTY;

  if (tcgetattr(terminal_descriptor, &terminal_original) ||
      tcgetattr(terminal_descriptor, &terminal_settings))
  {
    return errno = ENOTSUP;
  }

  if (isatty(STDIN_FILENO))
    setvbuf(stdin, NULL, _IONBF, 0);
  if (isatty(STDOUT_FILENO))
    setvbuf(stdout, NULL, _IONBF, 0);
  if (isatty(STDERR_FILENO))
    setvbuf(stderr, NULL, _IONBF, 0);

  if (atexit(terminal_done))
    return errno = ENOTSUP;

  struct sigaction act;
  sigemptyset(&act.sa_mask);
  act.sa_handler = terminal_signal;
  act.sa_flags = 0;
  if (sigaction(SIGHUP,  &act, NULL) ||
      sigaction(SIGINT,  &act, NULL) ||
      sigaction(SIGQUIT, &act, NULL) ||
      sigaction(SIGTERM, &act, NULL) ||
#ifdef SIGXCPU
      sigaction(SIGXCPU, &act, NULL) ||
#endif
#ifdef SIGXFSZ    
      sigaction(SIGXFSZ, &act, NULL) ||
#endif
#ifdef SIGIO
      sigaction(SIGIO,   &act, NULL) ||
#endif
      sigaction(SIGPIPE, &act, NULL) ||
      sigaction(SIGALRM, &act, NULL))
  {
    return errno = ENOTSUP;
  }

  terminal_settings.c_iflag &= ~IGNBRK;
  terminal_settings.c_iflag |=  BRKINT;
  terminal_settings.c_iflag |=  IGNPAR;
  terminal_settings.c_iflag &= ~PARMRK;
  terminal_settings.c_iflag &= ~ISTRIP;
  terminal_settings.c_cflag &= ~CSIZE;
  terminal_settings.c_cflag |=  CS8;
  terminal_settings.c_cflag |=  CREAD;
  terminal_settings.c_lflag |=  ISIG;
  terminal_settings.c_lflag &= ~ICANON;
  terminal_settings.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);
  terminal_settings.c_lflag &= ~IEXTEN;
  terminal_settings.c_cc[VTIME] = 0;
  terminal_settings.c_cc[VMIN] = 1;

  return errno = 0;
}

void set_terminal_raw_mode() {

  /* Set the new terminal settings.
   * Note that we don't actually check which ones were successfully
   * set and which not, because there isn't much we can do about it. */
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_settings);
}

void set_terminal_original_mode() {
  /* Restore original terminal settings. */
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_original);
}
 



bool wait_for_key_pressed(unsigned timeout_ms) {
  if (terminal_descriptor == -1) return false;

  struct pollfd pls[ 1 ];
  pls[ 0 ].fd     = terminal_descriptor;
  pls[ 0 ].events = POLLIN | POLLPRI;

  return poll( pls, 1, timeout_ms ) > 0;
}




float constrain(float input, float low, float high) {
  if(input<low) {
    input = low;
  }
  else if(input>high) {
    input = high;
  }
  else {
    input = input;
  }

  return input;
}

float makeSimpleProfile(float output, float input, float slop) {
  if(input>output) {
    output = min(input, output+slop);
  }
  else if(input<output) {
    output = max(input, output-slop);
  }
  else {
    output = input;
  }
  return output;
}

float checkLinearLimitVelocity(float vel) {
  vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL);
  return vel;
}

float checkAngularLimitVelocity(float vel) {
  vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL);
  return vel;
}

void new_velocity(float lin, float ang, bool direct) { // only set one of lin and ang

  geometry_msgs::Twist twist;

  if (direct) {

    if (lin != 0.0) { // direct, linear velocity control
      target_linear_vel = lin; control_linear_vel = lin;
      target_angular_vel = 0.0; control_angular_vel = 0.0;

    }
    else { // direct, angular velocity control
      target_linear_vel = 0.0; control_linear_vel = 0.0;
      target_angular_vel = ang; control_angular_vel = ang;
    }

  }
  else {

    if (lin == 1.0 || lin == -1.0) { // no direct, linear velocity control
      if (lin == 1.0) target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE);
      else target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE);
      control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0));
      control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
    }

    if (ang == 1.0 || ang == -1.0) { // no direct, angular velocity control
      if (ang == 1.0) target_angular_vel = checkAngularLimitVelocity(target_angular_vel+ANG_VEL_STEP_SIZE);
      else target_angular_vel = checkAngularLimitVelocity(target_angular_vel-ANG_VEL_STEP_SIZE);
      control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
      control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0));
    }

    if (lin == 0.0 && ang == 0.0) { // no direct, reset velocity
      target_linear_vel = 0.0; control_linear_vel = 0.0;
      target_angular_vel = 0.0; control_angular_vel = 0.0;
    }

  }

  twist.linear.x = control_linear_vel;
  twist.linear.y = 0.0; twist.linear.z = 0.0;
  twist.angular.x = 0.0; twist.angular.y = 0.0;
  twist.angular.z = control_angular_vel;
  vel_pub.publish(twist);

}




bool groupinit()
{
  // Moveit 
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Move group arm
  string planning_group_name = "arm";
  move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);

  // Move group gripper
  string planning_group_name2 = "gripper";
  move_group2_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name2);

  ros::start();
  return true;
}

bool position(vector<double> joint_angle, double path_time)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Next get the current set of joint values for the group.
  const robot_state::JointModelGroup* joint_model_group =
    move_group_->getCurrentState()->getJointModelGroup("arm");

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = joint_angle.at(0);  // radians
  joint_group_positions[1] = joint_angle.at(1);  // radians
  joint_group_positions[2] = joint_angle.at(2);  // radians
  joint_group_positions[3] = joint_angle.at(3);  // radians
  move_group_->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_->move();

  spinner.stop();
  return true;
}

bool gripper(vector<double> joint_angle)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Next get the current set of joint values for the group.
  const robot_state::JointModelGroup* joint_model_group =
    move_group2_->getCurrentState()->getJointModelGroup("gripper");

  moveit::core::RobotStatePtr current_state = move_group2_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = joint_angle.at(0);  // radians
  move_group2_->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group2_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group2_->move();

  spinner.stop();
  return true;
}

void update_cur_joint()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<double> jointValues = move_group_->getCurrentJointValues();
  std::vector<double> jointValues2 = move_group2_->getCurrentJointValues();
  std::vector<double> temp_angle;
  temp_angle.push_back(jointValues.at(0));
  temp_angle.push_back(jointValues.at(1));
  temp_angle.push_back(jointValues.at(2));
  temp_angle.push_back(jointValues.at(3));
  temp_angle.push_back(jointValues2.at(0));
  cur_joint = temp_angle;
}

void print_cur_joint()
{
  printf("///MSG///  ARM JOINT1: %f, JOINT2: %f, JOINT3: %f, JOINT4: %f\n",
                  cur_joint.at(0), cur_joint.at(1), cur_joint.at(2), cur_joint.at(3));
  printf("///MSG///  GRIPPER JOINT: %f\n", cur_joint.at(4));
}




void head_object(ros::NodeHandle node) {
  object_xmid = 0;
  object_sub = node.subscribe("/darknet_ros/bounding_boxes",100,msgCallback);
  new_velocity(0.0, 0.0, false);
  printf("head_object START\n");
  for (int i = 0; i < 2; i++) new_velocity(0.0, 1.0, false);

  ros::Rate loop_rate(10);
  clock_t start = clock();
  clock_t end = start;
  clock_t timeprint = end;

  
  while (!detected && double(end - start)/CLOCKS_PER_SEC <= 5.0) {

    ros::spinOnce();
    
    if (object_xmid > 660) {
      if (control_angular_vel > 0) {
        new_velocity(0.0, 0.0, false);
        for (int j = 0; j < 1; j++) new_velocity(0.0, -1.0, false);
      }
      else;
    }
    else if (object_xmid > 0 && object_xmid < 640) {
      new_velocity(0.0, 0.0, false);
      for (int j = 0; j < 1; j++) new_velocity(0.0, 1.0, false);
    }
    else if (object_xmid > 640 && object_xmid < 660) {
      new_velocity(0.0, 0.0, false);
      detected = true;
    }
    else;
    
    loop_rate.sleep();
    end = clock();
    if (double(end - timeprint)/CLOCKS_PER_SEC >= 5.0) {
      printf("///MSG///  WAIT --- OBJECT DETECTING TIME: %f\n", double(end - start)/CLOCKS_PER_SEC);
      timeprint = end;
    }
  }
  
  if (detected) printf("head_object COMPLETE\n");
  else printf("head_object FAIL\n");
  object_class = "None";
  object_xmid = 0;
  new_velocity(0.0, 0.0, false);
}


void head_object_right(ros::NodeHandle node) {
  object_xmid = 0;
  object_sub = node.subscribe("/darknet_ros/bounding_boxes",100,msgCallback);
  new_velocity(0.0, 0.0, false);
  printf("head_object START\n");
  for (int i = 0; i < 2; i++) new_velocity(0.0, -1.0, false);

  ros::Rate loop_rate(10);
  clock_t start = clock();
  clock_t end = start;
  clock_t timeprint = end;

  
  while (!detected && double(end - start)/CLOCKS_PER_SEC <= 15.0) {

    ros::spinOnce();
    
    if (object_xmid > 660) {
      if (control_angular_vel > 0) {
        new_velocity(0.0, 0.0, false);
        for (int j = 0; j < 1; j++) new_velocity(0.0, 1.0, false);
      }
      else;
    }
    else if (object_xmid > 0 && object_xmid < 640) {
      new_velocity(0.0, 0.0, false);
      for (int j = 0; j < 1; j++) new_velocity(0.0, -1.0, false);
    }
    else if (object_xmid > 640 && object_xmid < 660) {
      new_velocity(0.0, 0.0, false);
      detected = true;
    }
    else;
    
    loop_rate.sleep();
    end = clock();
    if (double(end - timeprint)/CLOCKS_PER_SEC >= 5.0) {
      printf("///MSG///  WAIT --- OBJECT DETECTING TIME: %f\n", double(end - start)/CLOCKS_PER_SEC);
      timeprint = end;
    }
  }
  
  if (detected) printf("head_object COMPLETE\n");
  else printf("head_object FAIL\n");
  object_class = "None";
  object_xmid = 0;
  new_velocity(0.0, 0.0, false);
}



void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{

  int xmin , xmax = 0;

  for(int i = 0; i < msg->bounding_boxes.size(); i++) {
    cout << "///MSG///  Bounding Boxes (Class):" << msg->bounding_boxes[i].Class << endl;
    if ((msg->bounding_boxes[i].Class == "bottle" || msg->bounding_boxes[i].Class == "vase") && msg->bounding_boxes[i].probability >= 0.6) {
      xmin = msg->bounding_boxes[i].xmin;
      xmax = msg->bounding_boxes[i].xmax;
      object_class = msg->bounding_boxes[i].Class;
      object_xmid = (xmin + xmax)/2;
      break;
    }
  }
}




void printText()
{
  printf("---------------------------\n");
  printf("---------------------------\n");
  printf("FINAL WHALE TURTLEBOT3 STATUS: %d \n", ttb_status);
  printf("---------------------------\n");
  printf("VEL LIN: %.3lf ANG: %.3lf\n",
         target_linear_vel, target_angular_vel);
  printf("ARM ANGLE J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         arm[0],
         arm[1],
         arm[2],
         arm[3]);
  printf("GRIPPER ANGLE: %.3lf\n", hand[0]);
  printf("---------------------------\n");
  printf("CURRENT POS X: %f, Y: %f ANG: %f\n", cur_pos_x, cur_pos_y, cur_pos_ang);
  printf("GOAL POS X: %f, Y: %f\n", goal_pos_x, goal_pos_y);
  printf("---------------------------\n");
  printf("---------------------------\n");
}




double get_distance_err() {
  return sqrt(pow(goal_pos_x - cur_pos_x, 2) + pow(goal_pos_y - cur_pos_y, 2));
}

double get_heading_err() {
  double deltaX = goal_pos_x - cur_pos_x;
  double deltaY = goal_pos_y - cur_pos_y;
  double Heading = atan2(deltaY, deltaX);
  double Heading_err = Heading - cur_pos_ang;

  // Make sure heading error falls within -PI to PI range
  if (Heading_err > PI) Heading_err -= (2 * PI);
  if (Heading_err < -PI) Heading_err += (2 * PI);
  return Heading_err;
}

void set_velocity_goal() {
  printf("CUR_POS: %f, %f, %f\n", cur_pos_x, cur_pos_y, cur_pos_ang);
  printf("GOAL_POS: %f, %f\n", goal_pos_x, goal_pos_y);

  double distance = get_distance_err();
  double angle = get_heading_err();

  if (!moving && (abs(distance) > distance_threshold)) {
    if (abs(angle) > angle_threshold) new_velocity(0.0, K_ang*angle, true);
    else new_velocity(K_lin*distance, 0.0, true);
  }
  else {
    new_velocity(0.0, 0.0, false);
    goal_pos_x = 0.0; goal_pos_y = 0.0;
    cur_pos_x = 0.0; cur_pos_y = 0.0; cur_pos_ang = 0.0;
    moving = true;
  }
}


void update_cur_pos (const nav_msgs::Odometry::ConstPtr& msg) {
  cur_pos_x = 10*msg->pose.pose.position.x;
  cur_pos_y = 10*msg->pose.pose.position.y;
  tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  cur_pos_ang = yaw;
}

void move_pos(vector<double> goal_pos, ros::NodeHandle node) {

  new_velocity(0.0, 0.0, false);
  goal_pos_x = goal_pos.front();
  goal_pos_y = goal_pos.back();
  printf("move_pos START\n");

  ros::Rate loop_rate(10);

  clock_t start = clock();
  clock_t end = start;
  clock_t timeprint = end;
  //curpos_sub = node.subscribe("odom",100,update_cur_pos);

  while (!moving && double(end - start)/CLOCKS_PER_SEC <= 30.0) {
    ros::spinOnce();
    set_velocity_goal();
    loop_rate.sleep();

    end = clock();
    if (double(end - timeprint)/CLOCKS_PER_SEC >= 5.0) {
      printf("///MSG/// WAIT --- POS MOVING TIME: %f\n", double(end - start)/CLOCKS_PER_SEC);
      timeprint = end;
    }
  }
  
  if (moving) printf("move_pos COMPLETE\n");
  else printf("move_pos FAIL\n");
  new_velocity(0.0, 0.0, false);
}


void move_reverse(ros::NodeHandle node) {
    new_velocity(0.0, 0.0, false);
    for (int i = 0; i < 4; i++) new_velocity(-1.0, 0.0, false);
    clock_t start = clock();
    clock_t end = start;
    while(true){
        if(double(end-start)/CLOCKS_PER_SEC >= 35){
        	break;
        }
        end = clock();
    }
    new_velocity(0.0, 0.0, false);
}


void move_forward(ros::NodeHandle node, bool go) {
    new_velocity(0.0, 0.0, false);
    float k = -1.0;
    if(go) {
      k = 1.0;
    }
    for (int i = 0; i < 4; i++) new_velocity(k, 0.0, false);
    clock_t start = clock();
    clock_t end = start;
    while(true){
        if(double(end-start)/CLOCKS_PER_SEC >= 10){
        	break;
        }
        end = clock();
    }
    new_velocity(0.0, 0.0, false);
}



void laser(const sensor_msgs::LaserScan::ConstPtr& msg) {

  int angle = 0;
  float distance = 0.0;

  int count = msg->scan_time / msg->time_increment;
  
  for(int i = 0; i < count; i++) {

    float degree = 180.0*(msg->angle_min + msg->angle_increment * i) / PI;
    angle = (int)degree;
    distance = msg->ranges[i];
    //printf("angle : %d, distance : %f\n", angle, distance);
    if ((angle >= 350 || angle <= 10) && distance < 0.30 && distance > 0.22) {
      approach = true;
      break;
    }
  }

}

void laser_max(const sensor_msgs::LaserScan::ConstPtr& msg) {

  is_spotted_ang = 0;
  float distance = 0.0;

  int count = msg->scan_time / msg->time_increment;
  
  for(int i = 0; i < count; i++) {
    float degree = 180.0*(msg->angle_min + msg->angle_increment * i) / PI;
    //printf("angle : %d, distance : %f\n", angle, distance);
    if(((int)degree > 0 && (int)degree < 90) && msg->ranges[i] > distance){
    	distance = msg->ranges[i];
    	is_spotted_ang = (int)degree;
    }
  }
  printf("distance : %f\n ", distance);
  if(distance < 0.5) is_spotted = true;
}

void spot_tracking(ros::NodeHandle node) {

  track_sub = node.subscribe("/scan",100,laser_max);
  new_velocity(0.0, 0.0, false);
  printf("spot_tracking START\n");

  ros::Rate loop_rate(10);
  clock_t start = clock();
  clock_t end = start;
  clock_t timeprint = end;
  is_spotted = false;

  while (!is_spotted) {

    ros::spinOnce();
    loop_rate.sleep();
    
    new_velocity(0.0, 0.0, false);
    int index = is_spotted_ang / 10;
    for (int i = 0; i < index / 45; i++) new_velocity(0.0, -1.0, false);
    clock_t start = clock();
    clock_t end = start;
    while(true){
        if(double(end-start)/CLOCKS_PER_SEC >= 1){
        	break;
        }
        end = clock();
    }    
    new_velocity(0.0, 0.0, false);
    for (int i = 0; i < 2; i++) new_velocity(1.0, 0.0, false);
    start = clock();
    end = start;
    while(true){
        if(double(end-start)/CLOCKS_PER_SEC >= 2){
        	break;
        }
        end = clock();
    }
  new_velocity(0.0, 0.0, false);
  }
  
  if (is_spotted) printf("spot_tracking COMPLETE\n");
  else printf("spot_tracking FAIL\n");
}


void head_tracking(ros::NodeHandle node) {

  track_sub = node.subscribe("/scan",100,laser);
  new_velocity(0.0, 0.0, false);
  printf("head_tracking START\n");
  for (int i = 0; i < 4; i++) new_velocity(1.0, 0.0, false);

  ros::Rate loop_rate(10);
  clock_t start = clock();
  clock_t end = start;
  clock_t timeprint = end;
  approach = false;

  while (!approach && double(end - start)/CLOCKS_PER_SEC <= 25.0) {

    ros::spinOnce();
    loop_rate.sleep();

    end = clock();
    if (double(end - timeprint)/CLOCKS_PER_SEC >= 0.3) {
      printf("///MSG///  WAIT --- OBJECT TRACKING TIME: %f\n", double(end - start)/CLOCKS_PER_SEC);
      timeprint = end;

      bool temp = detected; // to check whether it is right way to track the object
      detected = false;
      head_object(node);
      if (detected) {
        if (!approach) {
          for (int i = 0; i < 4; i++) new_velocity(1.0, 0.0, false);
        }
      }
      else approach = true;
      detected = temp;
    }
  }
  new_velocity(0.0, 0.0, false);
  if (approach) printf("head_tracking COMPLETE\n");
  else printf("head_tracking FAIL\n");
}




void hold_or_return(int choose) {

  printf("hold_or_return START\n");
  if (choose == 1) {
    if (!gripper(gr_open)) {
      cout << "[ERR] Failed to send" << endl;
    }
    usleep(2000*2000);
    position(ar_hold_1, 5.0);
    usleep(4000*4000);
    if (!gripper(/*gr_hold*/gr_hold)) {
      cout << "[ERR] Failed to send" << endl;
    }
    usleep(2000*2000);
    position(ar_hold_2, 5.0);
    usleep(4000*4000);
  }
  if (choose == 2) {
    position(ar_hold_1, 5.0);
    usleep(4000*4000);
    if (!gripper(gr_open)) {
      cout << "[ERR] Failed to send" << endl;
    }
    usleep(2000*2000);
    position(ar_home, 5.0);
    usleep(4000*4000);
  }
  if (choose == 3) {
    if (!gripper(gr_open)) {
      cout << "[ERR] Failed to send" << endl;
    }
    usleep(2000*2000);
    position(ar_hold_1, 5.0);
    usleep(4000*4000);
    if (!gripper(gr_hold)) {
      cout << "[ERR] Failed to send" << endl;
    }
    usleep(2000*2000);
    position(ar_home, 5.0);
    usleep(4000*4000);
  }
  printf("hold_or_return COMPLETE\n");
}
#endif
