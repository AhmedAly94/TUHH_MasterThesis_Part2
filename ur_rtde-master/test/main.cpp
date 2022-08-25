#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <vector>
#include <thread>
#include <chrono>

using namespace ur_rtde;

// Declare ur_rtde interfaces
std::shared_ptr<DashboardClient> db_client;
std::shared_ptr<RTDEControlInterface> rtde_control;
std::shared_ptr<RTDEReceiveInterface> rtde_receive;

// Declare initial values
std::vector<double> init_q;
std::vector<double> init_pose;

int main(int argc, char** argv) {
  doctest::Context context;

  context.setOption("abort-after", 5); // stop test execution after 5 failed assertions

  context.applyCommandLine(argc, argv);

  context.setOption("no-breaks", true); // don't break in the debugger when assertions fail

  // Power and brake release the robot through dashboard client
  db_client = std::make_shared<DashboardClient>("192.168.56.101", 29999, true);
  db_client->connect(5000);
  db_client->brakeRelease();

  // Wait for the brakes to release
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Initialize RTDE
  rtde_control = std::make_shared<RTDEControlInterface>("192.168.56.101");
  rtde_receive = std::make_shared<RTDEReceiveInterface>("192.168.56.101");
  init_q = rtde_receive->getActualQ();
  init_pose = rtde_receive->getActualTCPPose();

  int res = context.run(); // run test cases unless with --no-run

  if(context.shouldExit()) // query flags (and --exit) rely on the user doing this
    return res;            // propagate the result of the tests

  return res; // the result from doctest is propagated here as well
}

SCENARIO("move robot in joint space (moveJ)")
{
  GIVEN("a target joint configuration")
  {
    // Target is Pi / 6 in the robot base joint
    std::vector<double> target_q = init_q;
    target_q[0] += 0.5235; // ~ Pi / 6

    WHEN("robot is done moving")
    {
      REQUIRE(rtde_control->moveJ(target_q, 1.05, 1.4));

      THEN("robot must be at target")
      {
        std::vector<double> actual_q = rtde_receive->getActualQ();

        for(unsigned int i = 0; i < actual_q.size(); i++)
        {
          REQUIRE(actual_q[i] == doctest::Approx(target_q[i]).epsilon(0.005));
        }
      }
    }
  }
}

SCENARIO("move robot in tool space (moveL)")
{
  GIVEN("a cartesian target pose")
  {
    // Target 10 cm up in the Z-Axis of the TCP
    std::vector<double> target_pose = init_pose;
    target_pose[2] += 0.10; // ~ Pi / 6

    WHEN("robot is done moving")
    {
      REQUIRE(rtde_control->moveL(target_pose, 0.25, 0.5));

      THEN("robot must be at target")
      {
        std::vector<double> actual_tcp_pose = rtde_receive->getActualTCPPose();

        for(unsigned int i = 0; i < actual_tcp_pose.size(); i++)
        {
          REQUIRE(actual_tcp_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.005));
        }
      }
    }
  }
}
