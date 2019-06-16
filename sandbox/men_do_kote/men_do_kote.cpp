// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/gapi.hpp>
#include <opencv4/opencv2/gapi/core.hpp>
#include <opencv4/opencv2/gapi/imgproc.hpp>
#include <vector>
#include <numeric>

// Query frame size (width and height)
cv::Size get_cv_size(const rs2::video_frame &f) {
  return cv::Size(f.get_width(), f.get_height());
}
struct men_do_kote_t {
  std::vector<cv::Vec3f> mens;
  std::vector<std::vector<cv::Point> > dos;
  std::vector<std::vector<cv::Point> > kotes;
};
cv::Mat bgr2hsv(const cv::Mat& bgr) {
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV_FULL);
  return hsv;
}
men_do_kote_t get_men_do_kote(
    const cv::Mat& color_in,
    const cv::Mat& depth_in,
    const int near,
    const int far,
    cv::Mat& men_do_kote_visual
) {
  const double
    resize_scale = 0.2,
    resize_scale_inv = 1/resize_scale;
  const cv::GScalar   // HSV threshold for MEN, DO, and KOTE
    //                              H    S    V
    red_thresh_low1   (cv::Scalar(252, 128,  64)),
    red_thresh_up1    (cv::Scalar(255, 255, 255)),
    red_thresh_low2   (cv::Scalar(  0, 128,  64)),
    red_thresh_up2    (cv::Scalar(  3, 255, 255)),
    blue_thresh_low   (cv::Scalar(135, 128,  48)),
    blue_thresh_up    (cv::Scalar(175, 255, 255)),
    yellow_thresh_low (cv::Scalar( 28,  96,  64)),
    yellow_thresh_up  (cv::Scalar( 44, 255, 255));
  const cv::GScalar   // Depth threshold to distinguish opponent from background
    thresh_near       (near),
    thresh_far        (far);
  const auto kernel5 = cv::Mat::ones(5, 5, CV_8U)*255;
  const cv::GMat
    cin, din,
    // depth
    dresize   = cv::gapi::resize(din, cv::Size(), resize_scale, resize_scale),
    dvalid    = cv::gapi::inRange(dresize, thresh_near, thresh_far),
    blur_d    = cv::gapi::gaussianBlur(dvalid, cv::Size(7, 7), 2, 2),
    dmask     = cv::gapi::cmpGE(blur_d, cv::GScalar(32)),
    // color
    cresize   = cv::gapi::resize(cin, cv::Size(), resize_scale, resize_scale),
    red1      = cv::gapi::inRange(cresize, red_thresh_low1, red_thresh_up1),
    red2      = cv::gapi::inRange(cresize, red_thresh_low2, red_thresh_up2),
    red       = cv::gapi::bitwise_or(red1, red2),
    blue      = cv::gapi::inRange(cresize, blue_thresh_low , blue_thresh_up),
    yellow    = cv::gapi::inRange(cresize, yellow_thresh_low, yellow_thresh_up),
    masked_r  = cv::gapi::mask(red, dmask),
    masked_b  = cv::gapi::mask(blue, dmask),
    masked_y  = cv::gapi::mask(yellow, dmask),
    blur_r    = cv::gapi::gaussianBlur(masked_r, cv::Size(11, 11), 2, 2),
    blur_b    = cv::gapi::gaussianBlur(masked_b, cv::Size(15, 15), 2, 2),
    blur_y    = cv::gapi::gaussianBlur(masked_y, cv::Size(15, 15), 2, 2),
    bin_b     = cv::gapi::threshold(blur_b, cv::GScalar(16), cv::GScalar(255), cv::THRESH_BINARY),
    bin_y     = cv::gapi::threshold(blur_y, cv::GScalar(16), cv::GScalar(255), cv::THRESH_BINARY),
    merge     = cv::gapi::merge3(bin_b, bin_y, blur_r);
  // hsv
  //const cv::GMat h, s, v;
  //std::tie(h,s,v) = cv::gapi::split3(cresize);

  assert(color_in.size() == depth_in.size());

  cv::GComputation color2mdk(cv::GIn(cin, din), cv::GOut(blur_r, bin_b, bin_y, merge));

  cv::Mat men_im, do_im, kote_im;
  color2mdk.apply(cv::gin(bgr2hsv(color_in), depth_in), cv::gout(men_im, do_im, kote_im, men_do_kote_visual));

  men_do_kote_t mdk = {{}, {}, {}};

  // detect MEN
  const int dp=2, minDist=men_im.rows/8, canny_thresh_high=32, thresh_find=30, minRadius=6, maxRadius=30;
  HoughCircles(
      men_im, mdk.mens, cv::HOUGH_GRADIENT, dp,
      minDist, canny_thresh_high, thresh_find, minRadius, maxRadius);

  // detect DO and KOTE
  typedef std::vector<std::vector<cv::Point> > contours_t;
  contours_t do_contours, kote_contours;
  findContours(  do_im,   do_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  findContours(kote_im, kote_contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  const auto detect_do_kote = [](const auto &contours, auto &parts) {
    for (auto&& contour : contours) {
      // Approximate curves with lines
      const auto arc = cv::arcLength(contour, true);
      std::vector<cv::Point> approx;
      cv::approxPolyDP(cv::Mat(contour), approx, 0.05*arc, true);
      // Prune if area is too small
      if(cv::contourArea(approx) < 30) continue;
      // Prune if curve is not a rectangle
      if(approx.size() > 5) continue;
      // This is DO or KOTE
      parts.push_back(approx);
    }
  };

  detect_do_kote(  do_contours, mdk.dos);
  detect_do_kote(kote_contours, mdk.kotes);

  // Draw MEN
  for (auto&& c : mdk.mens) {
    const cv::Point center(cvRound(c[0]), cvRound(c[1]));
    const int radius = cvRound(c[2]);
    circle(men_do_kote_visual, center, radius, cv::Scalar(255,255,255), 1, 8, 0);
  }

  // Draw DO and KOTE
  cv::drawContours(men_do_kote_visual, mdk.dos,   -1, cv::Scalar(255,255,255));
  cv::drawContours(men_do_kote_visual, mdk.kotes, -1, cv::Scalar(255,255,255));

  // Re-scale
  for (auto&& c : mdk.mens) { c *= resize_scale_inv; }
  for (auto&& d : mdk.dos)  { for (auto&& dp : d) { dp*= resize_scale_inv; } }
  for (auto&& k : mdk.kotes){ for (auto&& kp : k) { kp*= resize_scale_inv; } }
  return mdk;
}

float get_depth_scale(rs2::device dev) {
  // Go over the device's sensors
  for (rs2::sensor& sensor : dev.query_sensors()) {
    // Check if the sensor if a depth sensor
    if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
      return dpt.get_depth_scale();
    }
  }
  throw std::runtime_error("Device does not have a depth sensor");
}


int main(int argc, char * argv[]) try {
  constexpr int FPS = 30;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;
  // Add desired streams to configuration
  cfg.disable_all_streams();
  cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8, FPS);
  cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16,  FPS);

  // Instruct pipeline to start streaming with the requested configuration
  auto profile = pipe.start(cfg);

  // Set short_range accurate preset
  const auto dsensor = profile.get_device().first<rs2::depth_sensor>();
  dsensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_SR300_VISUAL_PRESET_SHORT_RANGE);
  dsensor.set_option(RS2_OPTION_MOTION_RANGE, 2);
  dsensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);
  const float depth_scale = dsensor.get_depth_scale();

  // Set auto exposure and auto white balance
  //const auto csensor = profile.get_device().first<rs2::sr300_color_sensor>();
  //csensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
  //csensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1);
  //csensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);

  const auto visual_window = "MEN(red) DO(blue) KOTE(green) Visualizer";
  const auto color_window = "Color";
  const auto depth_window = "Depth";
  cv::namedWindow(color_window,   cv::WINDOW_AUTOSIZE);
  cv::namedWindow(depth_window,   cv::WINDOW_AUTOSIZE);
  cv::namedWindow(visual_window,  cv::WINDOW_AUTOSIZE);

  rs2::align  align(RS2_STREAM_COLOR);

  while (cv::waitKey(1)!='q') {
    // Wait for next set of frames from the camera
    const rs2::frameset     frameset  = align.process(pipe.wait_for_frames());
    const rs2::video_frame  color     = frameset.get_color_frame();
    const rs2::depth_frame  depth     = frameset.get_depth_frame();
    if (!color || !depth) continue;

    // Convert rs2::frame to cv::Mat
    const cv::Mat color_mat(get_cv_size(color), CV_8UC3,  (void*)color.get_data(), cv::Mat::AUTO_STEP);
    const cv::Mat depth_mat(get_cv_size(depth), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

    // Search datotsu-parts
    cv::Mat visual;
    const auto mdk = get_men_do_kote (
      color_mat, depth_mat,
      1, 1.0/depth_scale,
      visual
    );

    for (auto&& c : mdk.mens) {
      const cv::Point center(cvRound(c[0]), cvRound(c[1]));
      const int radius = cvRound(c[2]);

      // Draw circle
      cv::circle(color_mat, center, radius, cv::Scalar(0,255,255), 2, 8, 0);
      cv::circle(depth_mat, center, radius, cv::Scalar(255*256), 2, 8, 0);
    }
    cv::drawContours(color_mat, mdk.dos, -1, cv::Scalar(255,0,0));
    cv::drawContours(color_mat, mdk.kotes, -1, cv::Scalar(0,255,255));

    // Update the window with new data
    cv::imshow(color_window, color_mat);
    cv::imshow(depth_window, depth_mat);
    cv::imshow(visual_window, visual);
  }

  return EXIT_SUCCESS;
} catch (const rs2::error & e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}


