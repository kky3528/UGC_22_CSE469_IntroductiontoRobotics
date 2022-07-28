


#include "../include/final/final.h"

int main(int argc, char** argv) {

  //terminal_init();
  //set_terminal_raw_mode();

  ros::init(argc, argv, "TASK1");
  groupinit();
  ros::NodeHandle node;

  //setting for velocity
  vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  geometry_msgs::Twist twist;

  //setting for position
  curpos_sub = node.subscribe("odom",100,update_cur_pos);

  printText();

  //start code for task
  printf("START!!!\n");

  move_pos(pos_5, node);
  if (moving) moving = false;
  else return -1;

  head_object(node);
  if (detected) detected = false;
  else return -1;

  head_tracking(node);
  if (approach) approach = false;
  else return -1;

  hold_or_return(3);

  printf("END!!!\n");
  //end code for task
 

  //finalization
  gripper(gr_finish);
  position(ar_home, 5.0);
  arm = ar_home;
  hand = gr_finish;

  printText();

  //set_terminal_original_mode();
  //terminal_done();

  return 0;
}
