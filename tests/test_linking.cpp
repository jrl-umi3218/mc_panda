/** This creates an arbitrary panda module, the goal is to make sure there is
 * no undefined symbol in the generated RobotModule library */

#include <panda.h>

int main()
{
  mc_robots::PandaRobotModule robot(true, false, false);
  return 0;
}
