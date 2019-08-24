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
  cv::GComputation                color2mdk;

  bool                            uv_to_xyz(
      float xyz[3],
      const rs2::depth_frame& frame,
      int u,
      int v
    );

  static cv::Size                 get_cv_size(const rs2::video_frame &f);
  static cv::Point2f              contour_center(const std::vector<cv::Point> &contour);
  static cv::GComputation         gen_computation();
  static target_cand_t            conv_tct(const target_cand_t &tc);

  static constexpr double
    resize_scale            = 0.2,
    resize_scale_depth      = resize_scale*dec_magnitude,
    resize_scale_inv        = 1/resize_scale,
    resize_scale_inv_depth  = 1/resize_scale_depth;
  static constexpr float hard_offset[3] = {-0.18, 0.08, 0.48};

};

inline cv::Size Mikiri::get_cv_size(const rs2::video_frame &f) {
  return cv::Size(f.get_width(), f.get_height());
}

inline cv::Point2f Mikiri::contour_center(const std::vector<cv::Point> &contour) {
  cv::Point sum(0, 0);
  for (auto&& p : contour) { sum += p; }
  return ((cv::Point2f)sum)/((int)contour.size());
}

inline Mikiri::target_cand_t Mikiri::conv_tct(const target_cand_t &tc) {
  return {
    { tc.coord[2]+hard_offset[0],
      tc.coord[1]+hard_offset[1],
     -tc.coord[0]+hard_offset[2]},
    tc.area
  };
}

#endif /* end of include guard */
