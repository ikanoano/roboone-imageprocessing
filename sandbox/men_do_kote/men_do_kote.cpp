// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/gapi.hpp>
#include <opencv4/opencv2/gapi/core.hpp>
#include <opencv4/opencv2/gapi/imgproc.hpp>
#include <vector>

// Query frame size (width and height)
cv::Size get_cv_size(const rs2::frame &f) {
  const auto vf = f.as<rs2::video_frame>();
  return cv::Size(vf.get_width(), vf.get_height());
}
std::vector<cv::Vec3f> get_men(const cv::Mat& input, cv::Mat& output) {
  const double
    resize_scale = 0.2,
    resize_scale_inv = 1/resize_scale;
  const cv::GScalar  // YUV(YCbCr) threshold for red MEN
    //                      Y   Cb   Cr
    thresh_low(cv::Scalar( 20,  80, 170)),
    thresh_up (cv::Scalar(150, 130, 255));
  const auto kernel5 = cv::Mat::ones(5, 5, CV_8U)*255;
  const cv::GMat
    in,
    resize  = cv::gapi::resize(in, cv::Size(), resize_scale, resize_scale),
    yuv     = cv::gapi::BGR2YUV(resize),
    red     = cv::gapi::inRange(yuv, thresh_low, thresh_up),
    //erode   = cv::gapi::erode3x3(red),
    dilate  = cv::gapi::dilate3x3(red,  3),
    blur    = cv::gapi::gaussianBlur(dilate, cv::Size(7, 7), 2, 2),
    out     = blur;
  cv::GComputation color2blur(in, out);

  cv::Mat rslt;
  color2blur.apply(input, rslt);

  const int dp=2, minDist=rslt.rows/8, canny_thresh_high=32, thresh_find=30, minRadius=10, maxRadius=30;
  std::vector<cv::Vec3f> circles;
  HoughCircles(
      rslt, circles, cv::HOUGH_GRADIENT, dp,
      minDist, canny_thresh_high, thresh_find, minRadius, maxRadius);

  // Visualize
  cv::cvtColor(rslt, output, cv::COLOR_GRAY2BGR);
  for (auto&& c : circles) {
    const cv::Point center(cvRound(c[0]), cvRound(c[1]));
    const int radius = cvRound(c[2]);

    // Draw circle
    circle(output, center, radius, cv::Scalar(0,0,255), 1, 8, 0);
  }

  // Re-scale
  for (auto&& c : circles) {
    c[0] *= resize_scale_inv;
    c[1] *= resize_scale_inv;
    c[2] *= resize_scale_inv;
  }
  return circles;
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
    const auto circle_window = "Circle";
    const auto color_window = "Color";
    const auto depth_window = "Depth";
    namedWindow(color_window, WINDOW_AUTOSIZE);
    //namedWindow(depth_window, WINDOW_AUTOSIZE);
    namedWindow(circle_window, WINDOW_AUTOSIZE);

    while (waitKey(1)!='q') {
        const rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        const rs2::frame color = data.get_color_frame();
        const rs2::frame depth = data.get_depth_frame();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        const Mat color_mat(get_cv_size(color), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        const Mat depth_mat(get_cv_size(depth), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);

        // Search circle
        Mat intermediate;
        const auto circles = get_men(color_mat, intermediate);

        for (auto&& c : circles) {
          const cv::Point center(cvRound(c[0]), cvRound(c[1]));
          const int radius = cvRound(c[2]);

          // Draw circle
          circle(color_mat, center, radius, cv::Scalar(0,255,255), 2, 8, 0);
        }

        // Update the window with new data
        imshow(color_window, color_mat);
        //imshow(depth_window, depth_mat);
        imshow(circle_window, intermediate);
    }

    return EXIT_SUCCESS;
} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


