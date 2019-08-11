#include <cstdio>
#include "../src/OpponentUnit.hpp"

void printOpponent(const char* pre, const OpponentUnit::OpponentPart &op) {
  if(!op.has_value()) return;
  std::cout
    << pre << ": ("
    << op.value()[0] << ", "
    << op.value()[1] << ", "
    << op.value()[2] << ")"
    << std::endl;
}

int main(int argc, char * argv[]) try {

  OpponentUnit o;

  while (cv::waitKey(1)!='q') {
    const auto s = o.survey();
    printOpponent("men", s.men);
    printOpponent("dou", s.dou);
    printOpponent("kote", s.kote);
  }

  return EXIT_SUCCESS;
} catch (const rs2::error & e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
