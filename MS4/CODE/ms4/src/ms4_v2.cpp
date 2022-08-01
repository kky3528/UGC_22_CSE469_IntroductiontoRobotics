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
void new_velocity(int, int);

// for gripper and arm (manipulation)
const vector<double> gr_open = {0.010};
const vector<double> gr_close = {-0.010};
const vector<double> gr_finish = {0.000};
const vector<double> ar_home = {0.000, -0.963, 0.314, 0.680};
const vector<double> ar_init = {0.000, 0.031, 0.008, 0.000};
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
static vector<double> cur_joint = {0.000, 0.000, 0.000, 0.000, 0.000};


int main(int argc, char** argv) {

  terminal_init();
  set_terminal_raw_mode();

  bool isContinue = true;

  ros::init(argc, argv, "ms4_v1");
  groupinit();
  ros::NodeHandle node;

  //setting for velocity
  vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);   
  geometry_msgs::Twist twist;

  //setting for object detection
  
  while(isContinue) {
	  
    printText();

    while(true) {
      if(wait_for_key_pressed(100)) break;
    }
    char c = cin.get();
    if(c =='q') break;

    switch(c) {

      case 'w':
	ttb_status++;
        new_velocity(1, 0);
        break;
      case 'x':
	ttb_status++;
        new_velocity(-1, 0);
        break;
      case 'a':
	ttb_status++;
        new_velocity(0, 1);
        break;
      case 'd':
	ttb_status++;
        new_velocity(0, -1);
   	break;
      case 's':
	ttb_status++;
      	new_velocity(0, 0);
      	break;

      case 'g':
	ttb_status++;
	hand = gr_open;
        update_cur_joint();
        print_cur_joint();
	if (!gripper(gr_open)) {
	  cout << "[ERR] Failed to send" << endl;
	}
        update_cur_joint();
        print_cur_joint();

        break;
      case 'h':
	ttb_status++;
	hand = gr_close;
	update_cur_joint();
        print_cur_joint();
	if (!gripper(gr_close)) {
          cout << "[ERR] Failed to send" << endl;
        }
        update_cur_joint();
        print_cur_joint();
	break;

      case 'k':
	ttb_status++;
	arm = ar_home;
        update_cur_joint();
        print_cur_joint();
	position(ar_home, 5.0);
        update_cur_joint();
        print_cur_joint();
	break;
      case 'l':
	ttb_status++;
	arm = ar_init;
        update_cur_joint();
        print_cur_joint();
        position(ar_init, 5.0);
        update_cur_joint();
        print_cur_joint();
	break;
      
      case 'o':
	ttb_status++;
        head_object(node);
	break;

      default:
        break;
    }

  }

  //finalization
  gripper(gr_finish);
  position(ar_home, 5.0);
  arm = ar_home;
  hand = gr_finish;

  printText();

  set_terminal_original_mode();
  terminal_done();
  return 0;
}

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

void new_velocity(int lin, int ang) { // only set one of lin and ang

  geometry_msgs::Twist twist;

  if (lin == 1 || lin == -1) {
    if (lin == 1) target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE);
    else target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE);
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0));
    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
  }

  if (ang == 1 || ang == -1) {
    if (ang == 1) target_angular_vel = checkAngularLimitVelocity(target_angular_vel+ANG_VEL_STEP_SIZE);
    else target_angular_vel = checkAngularLimitVelocity(target_angular_vel-ANG_VEL_STEP_SIZE);
    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0));
  }

  if (lin == 0 && ang == 0) {
    target_linear_vel = 0.0; control_linear_vel = 0.0;
    target_angular_vel = 0.0; control_angular_vel = 0.0;
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
  printf("ARM JOINT1: %f, JOINT2: %f, JOINT3: %f, JOINT4: %f\n",
                  cur_joint.at(0), cur_joint.at(1), cur_joint.at(2), cur_joint.at(3));
  printf("GRIPPER JOINT: %f\n", cur_joint.at(4));
}




void head_object(ros::NodeHandle node) {

  object_sub = node.subscribe("/darknet_ros/bounding_boxes",100,msgCallback);
  new_velocity(0, 0);
  for (int i = 0; i < 2; i++) new_velocity(0, 1);

  ros::Rate loop_rate(10);
  clock_t start = clock();
  clock_t end = start;
  clock_t timeprint = end;

  while (!detected && double(end - start)/CLOCKS_PER_SEC <= 7.0) {


    ros::spinOnce();

    if (object_class == "None") {
    }
    else if (object_xmid > 660) {
      if (control_angular_vel >= 0) {
        new_velocity(0, 0);
        for (int j = 0; j < 1; j++) new_velocity(0, -1);
      }
      else;
    }
    else if (object_xmid < 640) {
      new_velocity(0, 0);
      for (int j = 0; j < 1; j++) new_velocity(0, 1);
    } 
    else if (object_xmid > 640 && object_xmid < 660) {
      new_velocity(0, 0);
      detected = true;
    }
    else;

    loop_rate.sleep();
    end = clock();
    if (double(end - timeprint)/CLOCKS_PER_SEC >= 5.0) {
      printf("WAIT --- OBJECT DETECTING TIME: %f\n", double(end - start)/CLOCKS_PER_SEC);
      timeprint = end;
    }
  }
  new_velocity(0, 0);
}

void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{

  int xmin , xmax = 0;

  for(int i = 0; i < msg->bounding_boxes.size(); i++) {
    //cout << "Bounding Boxes (Class):" << msg->bounding_boxes[i].Class << endl;
    if (msg->bounding_boxes[i].Class == "bottle" && msg->bounding_boxes[i].probability >= 0.6) {
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
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your OpenManipulator! STATUS: %d \n", ttb_status);
  printf("---------------------------\n");
  printf("w : increase LIN_VEL in task space\n");
  printf("x : decrease LIN_VEL in task space\n");
  printf("a : increase ANG_VEL in task space\n");
  printf("d : decrease ANG_VEL in task space\n");
  printf("s : halt in task space\n");
  printf("---------------------------\n");
  printf("\n");
  printf("g : open gripper\n");
  printf("h : close gripper\n");
  printf("k : home position\n");
  printf("l : init position\n");
  printf("---------------------------\n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("m to find the bottle\n");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         arm[0],
         arm[1],
         arm[2],
         arm[3]);
  printf("Present Velocity LIN: %.3lf ANG: %.3lf\n",
         target_linear_vel, target_angular_vel);
  printf("Present Gripper Angle: %.3lf\n", hand[0]);
  printf("---------------------------\n");
  printf("Object Detection: %d: \n", detected);
  if (detected) detected = false;
  printf("---------------------------\n");
}

