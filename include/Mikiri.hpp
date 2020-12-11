#ifndef MIKIRI_H
#define MIKIRI_H

#include <cmath>
#include <vector>
#include <array>
#include <thread>
#include <mutex>
#include <boost/optional.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/gapi.hpp>
#include "Mikagiri.hpp"

class Mikiri final : public Mikagiri {
public:
  Mikiri(int fps = 30, bool visualize = true);
                                  ~Mikiri();
  void                            start() override {};
  void                            stop()  override {};
  boost::optional<men_do_kote_t>  get_men_do_kote() override;

private:

  static constexpr int            dec_magnitude = 4;
  float                           depth_scale;

  std::mutex                      last_mdk_mutex;
  std::mutex                      visual_mutex;
  boost::optional<men_do_kote_t>  last_mdk;
  std::thread                     th;
  bool                            exit;
  rs2::pipeline                   pipe;
  rs2::config                     cfg;
  rs2::decimation_filter          dec_filter;
  rs2::spatial_filter             spat_filter;
  rs2::align                      align;
  cv::GComputation                color2mdk;
  time_stamp_t                    depth_timestamp_offset;
  bool                            visual_updated;
  cv::Mat                         visual;

  boost::optional<men_do_kote_t>  body();
  bool                            uv_to_xyz(
      float xyz[3],
      const rs2::depth_frame& frame,
      int u,
      int v
    );

  static cv::Size                 get_cv_size(const rs2::video_frame &f);
  static cv::Point2f              contour_center(const std::vector<cv::Point> &contour);
  static cv::GComputation         gen_computation(bool visualize);
  static target_cand_t            adj_center(const target_cand_t &tc, const float gap);
  static target_cand_t            conv_tct(const target_cand_t &tc);

  static constexpr double
    resize_scale            = 0.2,
    resize_scale_depth      = resize_scale*dec_magnitude,
    resize_scale_inv        = 1/resize_scale,
    resize_scale_inv_depth  = 1/resize_scale_depth;
  static constexpr float hard_offset[3] = {0.075, 0.132, 0.01};
  const char* visual_window = "MEN(red) DO(blue) KOTE(green) Visualizer";

  // surface to volume center gap in meter
  static constexpr float    men_s2c_gap   = 0.05 / 2;
  static constexpr float    dou_s2c_gap   = 0.14 / 2;
  static constexpr float    kote_s2c_gap  = 0.03 / 2;
};

inline cv::Size Mikiri::get_cv_size(const rs2::video_frame &f) {
  return cv::Size(f.get_width(), f.get_height());
}

inline cv::Point2f Mikiri::contour_center(const std::vector<cv::Point> &contour) {
  cv::Point sum(0, 0);
  for (auto&& p : contour) { sum += p; }
  return ((cv::Point2f)sum)/((int)contour.size());
}


#endif /* end of include guard */
