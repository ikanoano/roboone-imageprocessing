#include <cstdio>
#include "../src/OpponentUnit.hpp"

int main(int argc, char * argv[]) try {

  OpponentUnit o;

  while (cv::waitKey(1)!='q') {
    o.survey();
  }

  return EXIT_SUCCESS;
} catch (const rs2::error & e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
