// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <numeric>
#include <utility>
#include <tuple>
#include <librealsense2/rsutil.h>
#include <opencv2/gapi/core.hpp>
#include <opencv2/gapi/imgproc.hpp>
#include "../include/Mikiri.hpp"

// nonblocking
boost::optional<Mikiri::men_do_kote_t> Mikiri::get_men_do_kote () {
  if(visualize) {
    if(cv::waitKey(1)=='q') {exit=true;} // imshow needs waitKey to be updated
    std::lock_guard lock(visual_mutex);
    if(visual_updated) {cv::imshow(visual_window, visual);}
    visual_updated=false;
  }
  std::lock_guard lock(last_mdk_mutex);
  return std::exchange(last_mdk, boost::none);
}

boost::optional<Mikiri::men_do_kote_t> Mikiri::body () {
  constexpr int vthick = 3;

  // Wait for next set of frames from the camera
  rs2::frameset     frame_cd;
  if(!pipe.poll_for_frames(&frame_cd)) return boost::none;

  // Align & filtering
  rs2::frameset     aligned   = align.process(frame_cd);
  rs2::video_frame  color_rs  = aligned.get_color_frame();
  rs2::depth_frame  depth_rs  = aligned.get_depth_frame();
  if(!color_rs || !depth_rs) return boost::none;
  depth_rs = depth_rs.apply_filter(dec_filter);
  depth_rs = depth_rs.apply_filter(spat_filter);

  // Convert rs2::frame to cv::Mat
  const cv::Mat
    color_in(get_cv_size(color_rs), CV_8UC3,  (void*)color_rs.get_data(), cv::Mat::AUTO_STEP),
    depth_in(get_cv_size(depth_rs), CV_16UC1, (void*)depth_rs.get_data(), cv::Mat::AUTO_STEP);

  assert(color_in.size() == depth_in.size()*dec_magnitude);

  cv::Mat men_ext, do_ext, kote_ext, visual_ext;
  int green_area;
  color2mdk.apply(
    cv::gin(color_in, depth_in),
    visualize ? cv::gout(men_ext, do_ext, kote_ext, green_area, visual_ext) :
                cv::gout(men_ext, do_ext, kote_ext, green_area)
  );
  const int   refsize   = men_ext.size().height * men_ext.size().width / 12;
  const bool  startflag = green_area > refsize;

  std::vector<target_cand_t>  mens, dos, kotes;
  const auto annotate = [&](const float xyz[3], const cv::Point p) {
    char buf[128];
    sprintf(buf, "(%5.3f, %5.3f, %5.3f)", xyz[0], xyz[1], xyz[2]);
    cv::putText(visual_ext, buf, p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(  0,  0,  0), 7, cv::LINE_AA);
    cv::putText(visual_ext, buf, p, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2, cv::LINE_AA);
  };

  // detect MEN
  std::vector<cv::Vec3f>      mens_uvr; // { ((x,y), r), ... }
  const int dp=1, minDist=men_ext.rows/8, canny_thresh_high=32, thresh_find=10, minRadius=5, maxRadius=30;
  HoughCircles(
      men_ext, mens_uvr, cv::HOUGH_GRADIENT, dp,
      minDist, canny_thresh_high, thresh_find, minRadius, maxRadius);
  for (const auto& men_uvr : mens_uvr) {
    // Re-scale
    const auto men_uvr_color = men_uvr * resize_scale_inv;
    const auto men_uvr_depth = men_uvr * resize_scale_inv_depth;
    const cv::Point center(cvRound(men_uvr_color[0]), cvRound(men_uvr_color[1]));
    const int radius = cvRound(men_uvr_color[2]);
    // Prune if area is too small
    const int area = radius*radius*3;
    if(area < 40) continue;

    // Add
    float xyz[3];
    if(!uv_to_xyz(xyz, depth_rs, men_uvr_depth[0], men_uvr_depth[1])) continue;
    mens.push_back({{xyz[0], xyz[1], xyz[2]}, area});

    // Draw
    if(!visualize) continue;
    cv::circle(visual_ext, center, radius, cv::Scalar(224,224,255), vthick, 8, 0);
    annotate(xyz, center);
  }

  // detect DO and KOTE
  const auto detect_do_kote = [&](const auto &ext, auto &parts, const auto color) {
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(ext, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
      // Approximate curves with lines
      const auto arc = cv::arcLength(contour, true);
      std::vector<cv::Point> approx;
      cv::approxPolyDP(cv::Mat(contour), approx, 0.05*arc, true);

      // Prune if curve is not like a rectangle
      if(approx.size() < 3 || approx.size() > 5) continue;
      // Prune if contour is not convex
      if(!cv::isContourConvex(approx)) continue;
      // Prune if area is too small
      const int area = cv::contourArea(approx);
      if(area < 50) continue;

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
      if(!visualize) continue;
      std::vector<cv::Point> approxr;
      for (auto&& a : approx) { approxr.push_back(a*resize_scale_inv); }
      const std::vector<std::vector<cv::Point>> tmp = {approxr};
      cv::drawContours(visual_ext, tmp, -1, color, vthick);
      annotate(xyz, center_color);
    }
  };

  detect_do_kote(  do_ext, dos,   cv::Scalar(255,224,224));
  detect_do_kote(kote_ext, kotes, cv::Scalar(255,255,224));

  // Convert realsense coordinate system to actionplan's one
  for (auto&& e : mens)  e = conv_tct(e);
  for (auto&& e : dos)   e = conv_tct(e);
  for (auto&& e : kotes) e = conv_tct(e);

  if(visualize) {
    std::lock_guard lock(visual_mutex);
    visual = visual_ext;
    assert(visual.size().height>0 && visual.size().width>0);
    visual_updated = true;
  }

  const auto timestamp =
    depth_timestamp_offset + std::chrono::microseconds((long)(depth_rs.get_timestamp()*1000));
  return men_do_kote_t{timestamp, mens, dos, kotes, startflag};
}

bool Mikiri::uv_to_xyz(float xyz[3], const rs2::depth_frame& frame, const int u, const int v) {
    const int fwidth  = frame.get_width();
    const int fheight = frame.get_height();
    float dist = 0.0f;
    int validnum = 0;
    for (int du = -2; du <= 2; du++) {
      const int cu = u+du;
      if(cu<0 || cu>=fwidth) continue;
      for (int dv = -2; dv <= 2; dv++) {
        const int cv = v+dv;
        if(cv<0 || cv>=fheight) continue;
        const float tmp = frame.get_distance(cu, cv);
        if(tmp == 0.0f) continue;
        validnum++;
        dist += tmp;
      }
    }
    if(validnum==0) return false;
    dist /= validnum;

    // Deproject uv to xyz
    const rs2_intrinsics intr =
      frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    const float uv[] = {(float)u, (float)v};
    rs2_deproject_pixel_to_point(xyz, &intr, uv, dist);
    return true;
}

Mikiri::Mikiri(int fps, bool visualize) :
    Mikagiri(fps, visualize),
    last_mdk(boost::none),
    align(RS2_STREAM_COLOR),
    color2mdk(gen_computation(visualize)),
    visual_updated(false) {
  std::cout << "CV version: " << CV_VERSION          << std::endl;
  std::cout << "RS version: " << RS2_API_VERSION_STR << std::endl;

  // reset 
  {
    std::cout << "resetting device" << std::endl;
    rs2::context ctx;
    rs2::device dev = ctx.query_devices().front(); // Reset the first device
    dev.hardware_reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    rs2::device_hub hub(ctx);
    hub.wait_for_device();
    std::cout << "reset done" << std::endl;
  }

  // video stream
  cfg.disable_all_streams();
  cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, fps);
  cfg.enable_stream(RS2_STREAM_DEPTH,  640, 480, RS2_FORMAT_Z16,  fps);

  // filters
  dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_magnitude);
  spat_filter.set_option(RS2_OPTION_HOLES_FILL, 1); // 1 = 2 pix-radius fill

  // Instruct pipeline to start streaming with the requested configuration
  std::cout << "pipe.start " << std::endl;
  auto profile = pipe.start(cfg);
  rs2::frameset frame_cd;
  time_stamp_t timestamp; // volatile is not allowed
  std::cout << "polling for the first frame" << std::endl;
  do {
    timestamp = std::chrono::system_clock::now();
    // always false; prevent from optimizing timestamp
    if(depth_timestamp_offset > timestamp) break;
  } while (!pipe.poll_for_frames(&frame_cd));

  // Set short_range accurate preset
  const auto dsensor = profile.get_device().first<rs2::depth_sensor>();
  dsensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_SR300_VISUAL_PRESET_MID_RANGE);
  dsensor.set_option(RS2_OPTION_MOTION_RANGE, 2);
  dsensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);
  depth_scale = dsensor.get_depth_scale();

  // Set auto exposure and auto white balance
  rs2::video_frame color_rs = frame_cd.get_color_frame();
  const auto csensor = rs2::sensor_from_frame(color_rs);
  csensor->set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
  csensor->set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1);
  csensor->set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);

  // Set timestamp offset
  rs2::depth_frame depth_rs = frame_cd.get_depth_frame();
  double relative_timestamp_ms = depth_rs.get_timestamp();
  depth_timestamp_offset = timestamp -
    std::chrono::microseconds((long)(relative_timestamp_ms*1000));

  std::cout << "start visualizer" << std::endl;
  if(visualize) cv::namedWindow(visual_window,  cv::WINDOW_AUTOSIZE);

  // th is for separating the image processing thread from the main thread.
  // Not for interleaving a processing for each frame.
  // Keep it runs in 1/FPS seconds not to drop any frame.
  exit = false;
  th = std::thread([this]() {
    constexpr auto w = std::chrono::milliseconds(1);
    std::cout << "start thread" << std::endl;
    while(!exit) {
      const auto mdk_ = body();
      if(!mdk_) { // if mdk_ is none
        std::this_thread::sleep_for(w);
        continue;
      }
      std::lock_guard lock(last_mdk_mutex);
      last_mdk = mdk_;
    }
  });
}
Mikiri::~Mikiri() {
  exit = true;
  th.join();
}

cv::GComputation Mikiri::gen_computation(bool visualize) {
  // Initialize GComputation
  const cv::GScalar   // HSV threshold for MEN, DO, and KOTE
    //                              H    S    V
    red_thresh_low1   (cv::Scalar(168, 128,  48)),
    red_thresh_up1    (cv::Scalar(180, 255, 255)),
    red_thresh_low2   (cv::Scalar(  0, 128,  48)),
    red_thresh_up2    (cv::Scalar(  6, 255, 255)),
    blue_thresh_low   (cv::Scalar( 98, 128,  48)),
    blue_thresh_up    (cv::Scalar(128, 255, 255)),
    green_thresh_low  (cv::Scalar( 40,  96,  48)),
    green_thresh_up   (cv::Scalar( 75, 255, 208)),
    yellow_thresh_low (cv::Scalar( 20, 128, 160)),
    yellow_thresh_up  (cv::Scalar( 33, 255, 255));
  const cv::GMat
    bgrin, din;
  const auto [b, g, r] = cv::gapi::split3(bgrin);
  const cv::GMat
    // hsv
    hsvin     (cv::gapi::RGB2HSV(cv::gapi::merge3(r, g, b))),
    // depth
    dresize   (cv::gapi::resize(din, cv::Size(), resize_scale_depth, resize_scale_depth, cv::INTER_AREA)),
    dvalid    (cv::gapi::cmpGT(dresize, cv::GScalar(0))),
    blur_d    (cv::gapi::blur(dvalid, cv::Size(7, 7))),
    dmask     (cv::gapi::cmpGE(blur_d, cv::GScalar(4))),
    // color
    cresize   (cv::gapi::resize(hsvin, cv::Size(), resize_scale, resize_scale, cv::INTER_AREA)),
    red1      (cv::gapi::inRange(cresize, red_thresh_low1, red_thresh_up1)),
    red2      (cv::gapi::inRange(cresize, red_thresh_low2, red_thresh_up2)),
    red       (cv::gapi::bitwise_or(red1, red2)),
    blue      (cv::gapi::inRange(cresize, blue_thresh_low , blue_thresh_up)),
    yellow    (cv::gapi::inRange(cresize, yellow_thresh_low, yellow_thresh_up)),
    green     (cv::gapi::inRange(cresize, green_thresh_low, green_thresh_up)),
    masked_r  (cv::gapi::mask(red, dmask)),
    masked_b  (cv::gapi::mask(blue, dmask)),
    masked_y  (cv::gapi::mask(yellow, dmask)),
    blur_r    (cv::gapi::blur(masked_r, cv::Size( 7,  7))),
    blur_b    (cv::gapi::blur(masked_b, cv::Size( 5,  5))),
    blur_y    (cv::gapi::blur(masked_y, cv::Size( 5,  5))),
    bin_r     (std::get<0>(cv::gapi::threshold(blur_r, cv::GScalar(255), cv::THRESH_OTSU))),
    bin_b     (std::get<0>(cv::gapi::threshold(blur_b, cv::GScalar(255), cv::THRESH_OTSU))),
    bin_y     (cv::gapi::cmpGT(blur_y, cv::GScalar(127))),
    merge     (cv::gapi::merge3(bin_b, bin_y, bin_r)),
    // visualize
    mresize   (cv::gapi::resize(merge, cv::Size(), resize_scale_inv, resize_scale_inv)),
    dmresize  (cv::gapi::resize(dmask, cv::Size(), resize_scale_inv_depth, resize_scale_inv_depth)),
    visual    (cv::gapi::addWeighted(bgrin, 0.5, mresize, 0.75, 0.0));
  const cv::GOpaque<int>
    green_area(cv::gapi::countNonZero(green));

  return cv::GComputation(
    cv::GIn(bgrin, din),
    visualize ? cv::GOut(bin_r, bin_b, bin_y, green_area, visual) :
                cv::GOut(bin_r, bin_b, bin_y, green_area)
  );
}

Mikiri::target_cand_t Mikiri::conv_tct(const target_cand_t &tc) {
// 下がx+ coord[0] /上がx-
// 左がy+ coord[1] /右がy-
// 奥がz+ coord[2] /手前がz-
  const double a[3] = { -tc.coord[2], tc.coord[1], -tc.coord[0] };
// 右がx+
// 奥がy+
// 上がz+
  /*
     ______o_____   ^ y
     |__      __|   |
       |______|     0--> x
          |         z
  */
  // move realsense onto the center point of the rail
  const double b[3] = { a[0], a[1]+0.045, a[2] }; // y +45mm
  // head front
  constexpr double rad = -3.14159265358979323846*55/180; // -35 deg
  const double c[3] = { b[0]*std::cos(rad) - b[1]*std::sin(rad), b[0]*std::sin(rad) + b[1]*std::cos(rad), b[2] };
  // move realsense to jiki
  const double d[3] =   { c[0]-0.280, c[1]-0.110, c[2]+0.195 };
        // fix coordinate!!
  return {
    { d[1]+0.045,d[0]+0.520,d[2]-0.05},
    //{ a[0], a[1], a[2] },
    tc.area
  };
}
