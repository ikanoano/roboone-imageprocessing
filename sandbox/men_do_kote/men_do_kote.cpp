// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <opencv2/gapi/core.hpp>
#include <opencv2/gapi/imgproc.hpp>
#include <numeric>
#include <tuple>
#include "men_do_kote.hpp"

Mikiri::men_do_kote_t Mikiri::get_men_do_kote(
    cv::Mat&  men_do_kote_visual
) {
  constexpr int vthick = 3;

  // Wait for next set of frames from the camera
  rs2::frameset    frameset  = align.process(pipe.wait_for_frames());
  rs2::video_frame color_rs  = frameset.get_color_frame();
  rs2::depth_frame depth_rs  = frameset.get_depth_frame();
  while(!color_rs || !depth_rs) {
    // TODO: dirty
    frameset  = align.process(pipe.wait_for_frames());
    color_rs  = frameset.get_color_frame();
    depth_rs  = frameset.get_depth_frame();
  }
  depth_rs = depth_rs.apply_filter(dec_filter);
  depth_rs = depth_rs.apply_filter(spat_filter);

  // Convert rs2::frame to cv::Mat
  const cv::Mat
    color_in(get_cv_size(color_rs), CV_8UC3,  (void*)color_rs.get_data(), cv::Mat::AUTO_STEP),
    depth_in(get_cv_size(depth_rs), CV_16UC1, (void*)depth_rs.get_data(), cv::Mat::AUTO_STEP);

  assert(color_in.size() == depth_in.size()*dec_magnitude);

  cv::Mat men_ext, do_ext, kote_ext;
  color2mdk.apply(
    cv::gin(color_in, depth_in),
    cv::gout(men_ext, do_ext, kote_ext, men_do_kote_visual)
  );

  std::vector<target_cand_t>  mens, dos, kotes;
  const auto annotate = [&](const float xyz[3], const cv::Point p) {
    char buf[128];
    sprintf(buf, "(%5.3f, %5.3f, %5.3f)", xyz[0], xyz[1], xyz[2]);
    cv::putText(men_do_kote_visual, buf, p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(  0,  0,  0), 7, cv::LINE_AA);
    cv::putText(men_do_kote_visual, buf, p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2, cv::LINE_AA);
  };

  // detect MEN
  std::vector<cv::Vec3f>      mens_uvr; // { ((x,y), r), ... }
  const int dp=2, minDist=men_ext.rows/8, canny_thresh_high=32, thresh_find=30, minRadius=6, maxRadius=30;
  HoughCircles(
      men_ext, mens_uvr, cv::HOUGH_GRADIENT, dp,
      minDist, canny_thresh_high, thresh_find, minRadius, maxRadius);
  for (const auto& men_uvr : mens_uvr) {
    // Re-scale
    const auto men_uvr_color = men_uvr * resize_scale_inv;
    const auto men_uvr_depth = men_uvr * resize_scale_inv_depth;
    const cv::Point center(cvRound(men_uvr_color[0]), cvRound(men_uvr_color[1]));
    const int radius = cvRound(men_uvr_color[2]);
    const int area = radius*radius*3;

    // Add
    float xyz[3];
    if(!uv_to_xyz(xyz, depth_rs, men_uvr_depth[0], men_uvr_depth[1])) continue;
    mens.push_back({{xyz[0], xyz[1], xyz[2]}, area});

    // Draw
    cv::circle(men_do_kote_visual, center, radius, cv::Scalar(224,224,255), vthick, 8, 0);
    annotate(xyz, center);
  }

  // detect DO and KOTE
  const auto detect_do_kote = [&](const auto &ext, auto &parts, const auto color) {
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(ext, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
      // Approximate curves with lines
      const auto arc = cv::arcLength(contour, true);
      std::vector<cv::Point> approx;
      cv::approxPolyDP(cv::Mat(contour), approx, 0.05*arc, true);

      // Prune if curve is not like a rectangle
      if(approx.size() < 3 || approx.size() > 5) continue;
      // Prune if area is too small
      const int area = cv::contourArea(approx);
      if(area < 25) continue;

      // Re-scale
      const cv::Point2f
        center        = contour_center(approx);
      const cv::Point
        center_color  = center * resize_scale_inv,
        center_depth  = center * resize_scale_inv_depth;

      // Add
      float xyz[3];
      if(!uv_to_xyz(xyz, depth_rs, center_depth.x, center_depth.y)) continue;
      parts.push_back({
        {xyz[0], xyz[1], xyz[2]},
        (int)(area*resize_scale_inv*resize_scale_inv)
      });

      // Draw
      std::vector<cv::Point> approxr;
      for (auto&& a : approx) { approxr.push_back(a*resize_scale_inv); }
      const std::vector<std::vector<cv::Point>> tmp = {approxr};
      cv::drawContours(men_do_kote_visual, tmp, -1, color, vthick);
      annotate(xyz, center_color);
    }
  };

  detect_do_kote(  do_ext, dos,   cv::Scalar(255,224,224));
  detect_do_kote(kote_ext, kotes, cv::Scalar(255,255,224));

  return {mens, dos, kotes};
}

bool Mikiri::uv_to_xyz(float xyz[3], const rs2::depth_frame& frame, int u, int v) {
    constexpr int ds[][2] = {{0,0}, {0,1}, {0,-1}, {1,0}, {-1,0}};
    float dist;
    for (auto&& d : ds) {
      dist = frame.get_distance(u+d[0], v+d[1]);
      if(dist != 0.0f /*&& dist != -0.0f*/) break;
    }
    if(dist==0.0f) return false;

    // Deproject from pixel to xyz in 3D
    float xy[2] = {(float)u, (float)v};
    const rs2_intrinsics intr =
      frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    rs2_deproject_pixel_to_point(xyz, &intr, xy, dist);
    return true;
}

Mikiri::Mikiri(int fps) : FPS(fps), align(RS2_STREAM_COLOR), color2mdk(gen_computation()){
  std::cout << "CV version: " << CV_VERSION          << std::endl;
  std::cout << "RS version: " << RS2_API_VERSION_STR << std::endl;

  // video stream
  cfg.disable_all_streams();
  cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8, fps);
  cfg.enable_stream(RS2_STREAM_DEPTH, RS2_FORMAT_Z16,  fps);

  // filters
  dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_magnitude);
  spat_filter.set_option(RS2_OPTION_HOLES_FILL, 1); // 1 = 2 pix-radius fill

  // Instruct pipeline to start streaming with the requested configuration
  auto profile = pipe.start(cfg);

  // Set short_range accurate preset
  const auto dsensor = profile.get_device().first<rs2::depth_sensor>();
  dsensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_SR300_VISUAL_PRESET_MID_RANGE);
  dsensor.set_option(RS2_OPTION_MOTION_RANGE, 2);
  dsensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);
  depth_scale = dsensor.get_depth_scale();

  // Set auto exposure and auto white balance
  // TODO: how to get camera device?
  //const auto csensor = profile.get_device().first<rs2::sr300_color_sensor>();
  //csensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
  //csensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1);
  //csensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);
}

cv::GComputation Mikiri::gen_computation() {
  // Initialize GComputation
  const cv::GScalar   // HSV threshold for MEN, DO, and KOTE
    //                              H    S    V
    red_thresh_low1   (cv::Scalar(174, 128,  64)),
    red_thresh_up1    (cv::Scalar(180, 255, 255)),
    red_thresh_low2   (cv::Scalar(  0, 128,  64)),
    red_thresh_up2    (cv::Scalar(  6, 255, 255)),
    blue_thresh_low   (cv::Scalar( 95, 128,  48)),
    blue_thresh_up    (cv::Scalar(126, 255, 255)),
    yellow_thresh_low (cv::Scalar( 20, 128,  96)),
    yellow_thresh_up  (cv::Scalar( 30, 255, 255));
  const cv::GMat
    bgrin, din;
  const auto [b, g, r] = cv::gapi::split3(bgrin);
  const cv::GMat
    // hsv
    hsvin     (cv::gapi::RGB2HSV(cv::gapi::merge3(r, g, b))),
    // depth
    dresize   (cv::gapi::resize(din, cv::Size(), resize_scale_depth, resize_scale_depth, cv::INTER_AREA)),
    dvalid    (cv::gapi::cmpGT(dresize, cv::GScalar(0))),
    blur_d    (cv::gapi::gaussianBlur(dvalid, cv::Size(7, 7), 2, 2)),
    dmask     (cv::gapi::cmpGE(blur_d, cv::GScalar(16))),
    // color
    cresize   (cv::gapi::resize(hsvin, cv::Size(), resize_scale, resize_scale, cv::INTER_AREA)),
    red1      (cv::gapi::inRange(cresize, red_thresh_low1, red_thresh_up1)),
    red2      (cv::gapi::inRange(cresize, red_thresh_low2, red_thresh_up2)),
    red       (cv::gapi::bitwise_or(red1, red2)),
    blue      (cv::gapi::inRange(cresize, blue_thresh_low , blue_thresh_up)),
    yellow    (cv::gapi::inRange(cresize, yellow_thresh_low, yellow_thresh_up)),
    masked_r  (cv::gapi::mask(red, dmask)),
    masked_b  (cv::gapi::mask(blue, dmask)),
    masked_y  (cv::gapi::mask(yellow, dmask)),
    blur_r    (cv::gapi::gaussianBlur(masked_r, cv::Size(15, 15), 2, 2)),
    blur_b    (cv::gapi::gaussianBlur(masked_b, cv::Size( 7,  7), 2, 2)),
    blur_y    (cv::gapi::gaussianBlur(masked_y, cv::Size( 7,  7), 2, 2)),
    bin_b     (cv::gapi::threshold(blur_b, cv::GScalar(16), cv::GScalar(255), cv::THRESH_BINARY)),
    bin_y     (cv::gapi::threshold(blur_y, cv::GScalar(16), cv::GScalar(255), cv::THRESH_BINARY)),
    merge     (cv::gapi::merge3(bin_b, bin_y, blur_r)),
    mresize   (cv::gapi::resize(merge, cv::Size(), resize_scale_inv, resize_scale_inv)),
    visual    (cv::gapi::addWeighted(bgrin, 1.0, mresize, 1.0, 0.0));

  return cv::GComputation(
    cv::GIn(bgrin, din),
    cv::GOut(blur_r, bin_b, bin_y, visual)
  );
}

