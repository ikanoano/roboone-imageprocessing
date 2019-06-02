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
cv::Size get_cv_size(const rs2::frame &f) {
  const auto vf = f.as<rs2::video_frame>();
  return cv::Size(vf.get_width(), vf.get_height());
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
    const cv::Mat& input,
    cv::Mat& men_visual,
    cv::Mat& do_kote_visual
) {
  const double
    resize_scale = 0.2,
    resize_scale_inv = 1/resize_scale;
  const cv::GScalar  // HSV threshold for red MEN
    //                              H    S    V
    red_thresh_low1   (cv::Scalar(252, 128,  64)),
    red_thresh_up1    (cv::Scalar(255, 255, 255)),
    red_thresh_low2   (cv::Scalar(  0, 128,  64)),
    red_thresh_up2    (cv::Scalar(  3, 255, 255)),
    blue_thresh_low   (cv::Scalar(135, 128,  64)),
    blue_thresh_up    (cv::Scalar(175, 255, 255)),
    yellow_thresh_low (cv::Scalar( 28,  96,  64)),
    yellow_thresh_up  (cv::Scalar( 44, 255, 255));
  const auto kernel5 = cv::Mat::ones(5, 5, CV_8U)*255;
  const cv::GMat
    in,
    resize    = cv::gapi::resize(in, cv::Size(), resize_scale, resize_scale),
    red1      = cv::gapi::inRange(resize, red_thresh_low1, red_thresh_up1),
    red2      = cv::gapi::inRange(resize, red_thresh_low2, red_thresh_up2),
    red       = cv::gapi::bitwise_or(red1, red2),
    blue      = cv::gapi::inRange(resize, blue_thresh_low , blue_thresh_up),
    yellow    = cv::gapi::inRange(resize, yellow_thresh_low, yellow_thresh_up),
    blu_ylw   = cv::gapi::bitwise_or(blue, yellow),
    erode_by  = cv::gapi::erode3x3(blu_ylw),
    dilate_r  = cv::gapi::dilate3x3(red, 3),
    dilate_by = cv::gapi::dilate3x3(erode_by, 3),
    blur_r    = cv::gapi::gaussianBlur(red /*dilate_r*/, cv::Size(11, 11), 2, 2),
    blur_by   = cv::gapi::gaussianBlur(blu_ylw /*dilate_by*/, cv::Size(15, 15), 2, 2),
    thresh_by = cv::gapi::threshold(blur_by, cv::GScalar(16), cv::GScalar(255), cv::THRESH_BINARY);

  cv::GComputation color2blur(cv::GIn(in), cv::GOut(blue, blur_r, thresh_by));

  cv::Mat men_im, do_kote_im, blue_im;
  color2blur.apply(cv::gin(bgr2hsv(input)), cv::gout(blue_im, men_im, do_kote_im));

  men_do_kote_t mdk = {{}, {}, {}};

  // detect MEN
  const int dp=2, minDist=men_im.rows/8, canny_thresh_high=32, thresh_find=30, minRadius=10, maxRadius=30;
  HoughCircles(
      men_im, mdk.mens, cv::HOUGH_GRADIENT, dp,
      minDist, canny_thresh_high, thresh_find, minRadius, maxRadius);

  // detect DO and KOTE
  std::vector<std::vector<cv::Point> > contours;
  findContours(do_kote_im, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  for (auto&& contour : contours) {
    // Approximate curves with lines
    const auto arc = cv::arcLength(contour, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(cv::Mat(contour), approx, 0.01*arc, true);
    // Prune if area is too small
    //if(cv::contourArea(approx) < 50) continue;
    // Prune if curve is not a rectangle
    //if(approx.size() != 4) continue;
    // This is DO or KOTE
    const auto mean =
      std::accumulate(approx.begin(), approx.end(), cv::Point()) / (int)approx.size();
    if(blue_im.at<unsigned char>(mean)) {
      mdk.dos.push_back(approx);
    } else {
      mdk.kotes.push_back(approx);
    }
  }

  // Visualize
  cv::cvtColor(men_im, men_visual, cv::COLOR_GRAY2BGR);
  for (auto&& c : mdk.mens) {
    const cv::Point center(cvRound(c[0]), cvRound(c[1]));
    const int radius = cvRound(c[2]);

    // Draw circle
    circle(men_visual, center, radius, cv::Scalar(0,0,255), 1, 8, 0);
  }

  cv::cvtColor(do_kote_im, do_kote_visual, cv::COLOR_GRAY2BGR);
  // Draw contours
  cv::drawContours(do_kote_visual, mdk.dos, -1, cv::Scalar(255,0,0));
  cv::drawContours(do_kote_visual, mdk.kotes, -1, cv::Scalar(0,255,255));

  // Re-scale
  for (auto&& c : mdk.mens) { c *= resize_scale_inv; }
  for (auto&& d : mdk.dos)  { for (auto&& dp : d) { dp*= resize_scale_inv; } }
  for (auto&& k : mdk.kotes){ for (auto&& kp : k) { kp*= resize_scale_inv; } }
  return mdk;
}
int main(int argc, char * argv[]) try {
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    const rs2::align  align_to(RS2_STREAM_DEPTH);
    // Add desired streams to configuration
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16);

    // Instruct pipeline to start streaming with the requested configuration
    auto profile = pipe.start(cfg);

    // Set short_range accurate preset
    const auto sensor = profile.get_device().first<rs2::depth_sensor>();
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_SR300_VISUAL_PRESET_SHORT_RANGE);

    using namespace cv;
    const auto men_window = "MEN";
    const auto do_kote_window = "DO_KOTE";
    const auto color_window = "Color";
    const auto depth_window = "Depth";
    namedWindow(color_window, WINDOW_AUTOSIZE);
    //namedWindow(depth_window, WINDOW_AUTOSIZE);
    namedWindow(men_window, WINDOW_AUTOSIZE);
    namedWindow(do_kote_window, WINDOW_AUTOSIZE);

    while (waitKey(1)!='q') {
        const rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        const rs2::frame color = data.get_color_frame();
        const rs2::frame depth = data.get_depth_frame();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        const Mat color_mat(get_cv_size(color), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        const Mat depth_mat(get_cv_size(depth), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);

        // Search circle
        Mat men_visual, do_kote_visual;
        const auto mdk = get_men_do_kote(color_mat, men_visual, do_kote_visual);

        for (auto&& c : mdk.mens) {
          const cv::Point center(cvRound(c[0]), cvRound(c[1]));
          const int radius = cvRound(c[2]);

          // Draw circle
          circle(color_mat, center, radius, cv::Scalar(0,255,255), 2, 8, 0);
        }
        drawContours(color_mat, mdk.dos, -1, cv::Scalar(255,0,0));
        drawContours(color_mat, mdk.kotes, -1, cv::Scalar(0,255,255));

        // Update the window with new data
        imshow(color_window, color_mat);
        //imshow(depth_window, depth_mat);
        imshow(men_window, men_visual);
        imshow(do_kote_window, do_kote_visual);
    }

    return EXIT_SUCCESS;
} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


