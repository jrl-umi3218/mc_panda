/** This creates an arbitrary panda module, the goal is to make sure there is
 * no undefined symbol in the generated RobotModule library */

#include <mc_panda/panda.h>

int main()
{
  mc_panda::PandaRobotModule robot("panda_test_linking", true, false, false);
  return 0;
}
