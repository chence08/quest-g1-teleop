// Exercises the production serial/hand headers against a PTY, never hardware.
#include "inspire.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>

void require(bool value, const char* message)
{
  if(!value) throw std::runtime_error(message);
}

int main(int argc, char** argv)
{
  try
  {
    require(argc == 4, "usage: client /dev/pts/N mode cycles");
    const std::string path(argv[1]), mode(argv[2]);
    require(path.rfind("/dev/pts/", 0) == 0, "test client accepts PTYs only");
    const int cycles = std::stoi(argv[3]);
    auto serial = std::make_shared<SerialPort>(path, B115200);
    inspire::InspireHand hand(serial, 1);
    for(int cycle = 0; cycle < cycles; ++cycle)
    {
      Eigen::Matrix<double, 6, 1> target, measured;
      for(int joint = 0; joint < 6; ++joint) target(joint) = (200 + cycle + joint * 10) / 1000.;
      measured.setConstant(0.42);
      const auto before = std::chrono::steady_clock::now();
      if(mode == "success")
      {
        require(hand.SetPosition(target) == 0, "valid write ACK rejected");
        require(hand.GetPosition(measured) == 0, "valid state rejected");
        for(int joint = 0; joint < 6; ++joint)
          require(std::abs(measured(joint) - (300 + cycle + joint * 10) / 1000.) < 1e-12,
                  "wrong frame accepted as position feedback");
      }
      else if(mode == "bad_ack")
      {
        require(hand.SetPosition(target) != 0, "invalid write ACK accepted");
      }
      else if(mode == "invalid_target")
      {
        target(0) = std::numeric_limits<double>::quiet_NaN();
        require(hand.SetPosition(target) == 5, "nonfinite target accepted");
      }
      else
      {
        require(hand.GetPosition(measured) != 0, "invalid/missing state accepted");
        require((measured.array() == 0.42).all(), "failed read changed feedback");
      }
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - before).count();
      if(mode != "success" && mode != "invalid_target")
      {
        require(elapsed >= 0.035, "transaction deadline was reused or shortened");
        require(elapsed < 0.15, "transaction exceeded bounded deadline");
      }
    }
    std::cout << "PASS " << mode << " " << cycles << " cycles\n";
    return 0;
  }
  catch(const std::exception& error)
  {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
