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
// head delete

static int terminal_descriptor = -1;
static struct termios terminal_original;
static struct termios terminal_settings;
static char UP_ARROW_KEY = '8';
static char DOWN_ARROW_KEY = '2';
static char LEFT_ARROW_KEY = '4';
static char RIGHT_ARROW_KEY = '6';
static char ESC_KEY = 'q';


//#include "../include/ms3/posix_terminal.hpp"

using namespace std;

// This code is based on 
//   https://cboard.cprogramming.com/linux-programming/158476-termios-examples.html
// and 
//   http://www.cplusplus.com/forum/general/5304/#msg23940

// Usage:
//
//   ....
//   terminal_init();
//   ....
//   set_terminal_raw_mode();
//   ....
//   bool isContinue = true;
//   while(isContinue) {
//     ....    
//     while(true) {
//       if (wait_for_key_pressed(100)) break;
//       ....
//     }
//     ....
//     char c = getKeyStroke();
//     switch(c) {
//       ....
//       case 'q':
//         isContinue = false;
//         break;
//       ....
//     }
//   }
//   ....
//   set_terminal_original_mode();


static const float WAFFLE_MAX_LIN_VEL = 0.26;
static const float WAFFLE_MAX_ANG_VEL = 1.82;
static const float LIN_VEL_STEP_SIZE = 0.01;
static const float ANG_VEL_STEP_SIZE = 0.1;
int terminal_init() ;
void set_terminal_raw_mode () ;
void set_terminal_original_mode ();
bool wait_for_key_pressed(unsigned );
char getKeyStroke() ;
moveit::planning_interface::MoveGroupInterface* move_group_;
moveit::planning_interface::MoveGroupInterface* move_group2_;


bool ydyinit()
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

bool ahffkdy(vector<double> kinematics_pose, double path_time)
{
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  // Uncomment below to keep the end-effector parallel to the ground
  /*
  moveit_msgs::OrientationConstraint oc;
  oc.link_name = "end_effector_link";
  oc.header.frame_id = "link1";
  oc.orientation.w = 1.0;
  oc.absolute_x_axis_tolerance = 0.1;
  oc.absolute_y_axis_tolerance = 0.1;
  oc.absolute_z_axis_tolerance = 3.14;
  oc.weight = 1.0;
  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(oc);
  move_group_->setPathConstraints(constraints);
  */

  geometry_msgs::Pose target_pose;
  target_pose.position.x = kinematics_pose.at(0);
  target_pose.position.y = kinematics_pose.at(1);
  target_pose.position.z = kinematics_pose.at(2);
  // move_group_->setPoseTarget(target_pose); // Cannot use setPoseTarget as the robot has only 4DOF.
  move_group_->setPositionTarget(
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_->move();

  spinner.stop();
  return true;
}

bool gripper(std::vector<double> joint_angle)
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

void printText(vector<double> arm, vector<double> hand, float target_linear_vel, float target_angular_vel)
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your OpenManipulator!\n");
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
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         arm[0],
         arm[1],
         arm[2],
         arm[3]);
  printf("Present Velocity LIN: %.3lf ANG: %.3lf\n",
         target_linear_vel, target_angular_vel);
  printf("Present Gripper Angle: %.3lf\n", hand[0]);
  printf("---------------------------\n");
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


void vels(float target_linear_vel, float target_angular_vel) {
  cout<<"currently:\tlinear vel "<<target_linear_vel<<"\t"<<"angular vel "<<target_angular_vel << endl;

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




int main(int argc, char** argv) {
  terminal_init();
  set_terminal_raw_mode();
  bool isContinue = true;
  ros::init(argc, argv, "ms3_v1");
  ydyinit();
  ros::NodeHandle ph_;
  // ph_("~");
  ros::Publisher vel_pub_;
  vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

  int status = 0;
  float target_linear_vel = 0.0;
  float target_angular_vel = 0.0;
  float control_linear_vel = 0.0;
  float control_angular_vel = 0.0;   

  geometry_msgs::Twist twist;
  vector<double> open = {0.010};
  vector<double> close = {-0.010};
  vector<double> finish = {0.000};
  vector<double> home = {0.000, -0.963, 0.314, 0.680};
  vector<double> init = {0.000, 0.031, 0.008, 0.000};
  vector<double> arm = home;
  vector<double> hand = finish;

  while(isContinue) {
    printText(arm, hand, target_linear_vel, target_angular_vel);

    while(true) {
      if(wait_for_key_pressed(100)) break;
    }
    char c = getKeyStroke();

    if(c=='q') break;

    switch(c) {

      case 'w':
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel+LIN_VEL_STEP_SIZE);
        status = status + 1;
 
        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel,(LIN_VEL_STEP_SIZE/2.0));
        twist.linear.x = control_linear_vel;
        twist.linear.y = 0.0; twist.linear.z = 0.0;

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
        twist.angular.x = 0.0; twist.angular.y = 0.0;
        twist.angular.z = control_angular_vel;
        vel_pub_.publish(twist);
        break;

      case 'x':
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE);
        status = status + 1;
        //vels(target_linear_vel, target_angular_vel);
    
        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel,(LIN_VEL_STEP_SIZE/2.0));
        twist.linear.x = control_linear_vel;
        twist.linear.y = 0.0; twist.linear.z = 0.0;

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
        twist.angular.x = 0.0; twist.angular.y = 0.0;
        twist.angular.z = control_angular_vel;
        vel_pub_.publish(twist);
        break;

      case 'a':
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel+ANG_VEL_STEP_SIZE);
        status = status + 1;
        //vels(target_linear_vel, target_angular_vel);
      
        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel,(LIN_VEL_STEP_SIZE/2.0));
        twist.linear.x = control_linear_vel;
        twist.linear.y = 0.0; twist.linear.z = 0.0;

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
        twist.angular.x = 0.0; twist.angular.y = 0.0;
        twist.angular.z = control_angular_vel;
        vel_pub_.publish(twist);
        break;

      case 'd':
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel-ANG_VEL_STEP_SIZE);
        status = status+1;
        //vels(target_linear_vel, target_angular_vel);
      
        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel,(LIN_VEL_STEP_SIZE/2.0));
        twist.linear.x = control_linear_vel;
        twist.linear.y = 0.0; twist.linear.z = 0.0;

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0));
        twist.angular.x = 0.0; twist.angular.y = 0.0;
        twist.angular.z = control_angular_vel;
        vel_pub_.publish(twist);
   	break;

      case 's':
      	target_linear_vel = 0.0;
      	target_angular_vel = 0.0;
      	status = status + 1;
      	//vels(0.0 , 0.0);
     
        control_linear_vel = 0.0;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0; twist.linear.z = 0.0;

        control_angular_vel = 0.0;
        twist.angular.x = 0.0; twist.angular.y = 0.0;
        twist.angular.z = 0.0;
        vel_pub_.publish(twist);
      	break;

      case 'g':
	hand = open;
	if (!gripper(open)) {
	  cout << "[ERR] Failed to send" << endl;
	  break;
	}
        break;

      case 'h':
	hand = close;
	if (!gripper(close)) {
          cout << "[ERR] Failed to send" << endl;
          break;
        }
	break;

      case 'k':
	arm = home;
	position(home, 5.0);
	break;

      case 'l':
	arm = init;
        position(init, 5.0);
	break;

      default:
        break;
    }

  }


 //finalization
 gripper(finish);
 position(home, 5.0);
 arm = home;
 hand = finish;
 printText(arm, hand, target_linear_vel, target_angular_vel);
 set_terminal_original_mode();

 return 0;
}

/* Restore terminal to original settings */
void terminal_done() {
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_original);
}
 

/* "Default" signal handler: restore terminal, then exit.  */
void terminal_signal(int signum) {
  cout << "Terminal_signal(" << signum << ")" << endl;
  if (terminal_descriptor != -1)
    tcsetattr(terminal_descriptor, TCSANOW, &terminal_original);
  /* exit() is not async-signal safe, but _exit() is.
   * Use the common idiom of 128 + signal number for signal exits.
   * Alternative approach is to reset the signal to default handler,
   * and immediately raise() it. */
  _exit(128 + signum);
}
 

/* 
 * Initialize terminal for non-canonical, non-echo mode,
 * that should be compatible with standard C I/O.
 * Returns 0 if success, nonzero errno otherwise.
*/
int terminal_init() {

  /* Already initialized? */
  if (terminal_descriptor != -1)
    return errno = 0;

  /* Which standard stream is connected to our TTY? */
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

  /* Obtain terminal settings. */
  if (tcgetattr(terminal_descriptor, &terminal_original) ||
      tcgetattr(terminal_descriptor, &terminal_settings))
  {
    return errno = ENOTSUP;
  }

  // Disable buffering for terminal streams.
  if (isatty(STDIN_FILENO))
    setvbuf(stdin, NULL, _IONBF, 0);
  if (isatty(STDOUT_FILENO))
    setvbuf(stdout, NULL, _IONBF, 0);
  if (isatty(STDERR_FILENO))
    setvbuf(stderr, NULL, _IONBF, 0);

  /* At exit() or return from main(),
   * restore the original settings. */
  if (atexit(terminal_done))
    return errno = ENOTSUP;

  /* Set new "default" handlers for typical signals,
   * so that if this process is killed by a signal,
   * the terminal settings will still be restored first. */
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

  /* Let BREAK cause a SIGINT in input. */
  terminal_settings.c_iflag &= ~IGNBRK;
  terminal_settings.c_iflag |=  BRKINT;
 
  /* Ignore framing and parity errors in input. */
  terminal_settings.c_iflag |=  IGNPAR;
  terminal_settings.c_iflag &= ~PARMRK;
 
  /* Do not strip eighth bit on input. */
  terminal_settings.c_iflag &= ~ISTRIP;
 
  /* Do not do newline translation on input. */
  // terminal_settings.c_iflag &= ~(INLCR | IGNCR | ICRNL);
 
#ifdef IUCLC
  /* Do not do uppercase-to-lowercase mapping on input. */
  terminal_settings.c_iflag &= ~IUCLC;
#endif
 
  /* Use 8-bit characters. This too may affect standard streams,
   * but any sane C library can deal with 8-bit characters. */
  terminal_settings.c_cflag &= ~CSIZE;
  terminal_settings.c_cflag |=  CS8;

  /* Enable receiver. */
  terminal_settings.c_cflag |=  CREAD;

  /* Let INTR/QUIT/SUSP/DSUSP generate the corresponding signals. */
  terminal_settings.c_lflag |=  ISIG;

  /* Enable noncanonical mode.
   * This is the most important bit, as it disables line buffering etc. */
  terminal_settings.c_lflag &= ~ICANON;

  /* Disable echoing input characters. */
  terminal_settings.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);

  /* Disable implementation-defined input processing. */
  terminal_settings.c_lflag &= ~IEXTEN;

  /* To maintain best compatibility with normal behaviour of terminals,
   * we set TIME=0 and MAX=1 in noncanonical mode. This means that
   * read() will block until at least one byte is available. */
  terminal_settings.c_cc[VTIME] = 0;
  terminal_settings.c_cc[VMIN] = 1;

  /* Done. */
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
 

bool wait_for_key_pressed(unsigned timeout_ms /* = 0 */) {
  if (terminal_descriptor == -1) return false;

  struct pollfd pls[ 1 ];
  pls[ 0 ].fd     = terminal_descriptor;
  pls[ 0 ].events = POLLIN | POLLPRI;

  return poll( pls, 1, timeout_ms ) > 0;
}


char getKeyStroke() {
  while(true) {
    char c = cin.get();
    if (c != 27) {
      return c;
    } else if (wait_for_key_pressed(50)) {
      c = cin.get();
      if (c == '[') {
        c = cin.get();
        switch(c) {
          case 'A':
            return UP_ARROW_KEY;
          case 'B':
            return DOWN_ARROW_KEY;
          case 'C':
            return LEFT_ARROW_KEY;
          case 'D':
            return RIGHT_ARROW_KEY;
          default:
            return ESC_KEY;
        }
      } else {
        return ESC_KEY;
      }
    } else {
      return ESC_KEY;
    }
  }
}


