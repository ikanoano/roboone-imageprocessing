#ifndef MEN_DO_KOTE
#define MEN_DO_KOTE

#include <vector>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>
#include <array>

class Mikiri {

public:
  const int FPS;

  struct target_cand_t {
    std::array<float, 3>  coord;
    int                   area;
  };
  struct men_do_kote_t {
    std::vector<target_cand_t> mens;
    std::vector<target_cand_t> dos;
    std::vector<target_cand_t> kotes;
  };

  Mikiri(int fps = 30);
  void                            start() {}; // To be implemented
  void                            stop()  {}; // To be implemented
  men_do_kote_t                   get_men_do_kote(cv::Mat&  men_do_kote_visual);

private:

  static constexpr int            dec_magnitude = 2;
  float                           depth_scale;

  rs2::pipeline                   pipe;
  rs2::config                     cfg;
  rs2::decimation_filter          dec_filter;
  rs2::spatial_filter             spat_filter;
  rs2::align                      align;

  bool                            uv_to_xyz(
      float xyz[3],
      const rs2::depth_frame& frame,
      int u,
      int v
    );

  static cv::Size                 get_cv_size(const rs2::video_frame &f);
  static cv::Point                contour_center(std::vector<cv::Point> contour);
  static cv::Mat                  bgr2hsv(const cv::Mat& bgr);
  static float                    get_depth_scale(rs2::device dev);

  static constexpr double
    resize_scale            = 0.2,
    resize_scale_depth      = resize_scale*dec_magnitude,
    resize_scale_inv        = 1/resize_scale,
    resize_scale_inv_depth  = 1/resize_scale_depth;
  const cv::GScalar   // HSV threshold for MEN, DO, and KOTE
    //                              H    S    V
    red_thresh_low1,
    red_thresh_up1,
    red_thresh_low2,
    red_thresh_up2,
    blue_thresh_low,
    blue_thresh_up,
    yellow_thresh_low,
    yellow_thresh_up;
  const cv::GMat
    bgrin, hsvin, din,
    // depth
    dresize,
    dvalid,
    blur_d,
    dmask,
    // color
    cresize,
    red1,
    red2,
    red,
    blue,
    yellow,
    masked_r,
    masked_b,
    masked_y,
    blur_r,
    blur_b,
    blur_y,
    bin_b,
    bin_y,
    merge,
    mresize,
    visual;
  cv::GComputation color2mdk;
  // hsv
  //const cv::GMat h, s, v;
  //std::tie(h,s,v) = cv::gapi::split3(cresize);
};

inline cv::Size Mikiri::get_cv_size(const rs2::video_frame &f) {
  return cv::Size(f.get_width(), f.get_height());
}

inline cv::Point Mikiri::contour_center(std::vector<cv::Point> contour) {
  float x=0, y=0;
  for (auto&& p : contour) { x+= p.x; y+=p.y; }
  return {(int)(x/contour.size()), (int)(y/contour.size())};
}

inline cv::Mat Mikiri::bgr2hsv(const cv::Mat& bgr) {
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV_FULL);
  return hsv;
}

inline float get_depth_scale(rs2::device dev) {
  // Go over the device's sensors
  for (rs2::sensor& sensor : dev.query_sensors()) {
    // Check if the sensor if a depth sensor
    if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
      return dpt.get_depth_scale();
    }
  }
  throw std::runtime_error("Device does not have a depth sensor");
}

#endif /* end of include guard */
