#include <cstdio>
#include "men_do_kote.hpp"

int main(int argc, char * argv[]) try {

  const auto visual_window = "MEN(red) DO(blue) KOTE(green) Visualizer";
  cv::namedWindow(visual_window,  cv::WINDOW_AUTOSIZE);

  Mikiri m;

  while (cv::waitKey(1)!='q') {

    // Search datotsu-parts
    cv::Mat visual_mat;
    const auto mdk = m.get_men_do_kote(visual_mat);

    // Update the window with new data
    cv::imshow(visual_window, visual_mat);
  }

  return EXIT_SUCCESS;
} catch (const rs2::error & e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
