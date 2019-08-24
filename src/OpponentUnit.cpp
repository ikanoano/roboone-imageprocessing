#include <algorithm>
#include "OpponentUnit.hpp"
#include "Mikiri.hpp"

OpponentUnit::OpponentUnit(int fps) : m(fps) {
  cv::namedWindow(visual_window,  cv::WINDOW_AUTOSIZE);
}

void OpponentUnit::startCamera() {} // To be implemented
void OpponentUnit::stopCamera() {}  // To be implemented
OpponentUnit::OpponentModel OpponentUnit::survey() {
  cv::Mat visual_mat;
  auto mdk = m.get_men_do_kote(visual_mat);

  cv::Point2f src_center(visual_mat.cols/2.0F, visual_mat.rows/2.0F);
  cv::Mat rot_mat = getRotationMatrix2D(src_center, -90, 1.0);
  rot_mat.at<double>(0,2) += -visual_mat.cols/2 + visual_mat.rows/2;
  rot_mat.at<double>(1,2) += -visual_mat.rows/2 + visual_mat.cols/2;
  cv::Mat dst;
  cv::warpAffine(visual_mat, dst, rot_mat, cv::Size(visual_mat.rows,visual_mat.cols));

  cv::imshow(visual_window, dst);

  // pick largest target
  constexpr auto howtosort =
    [](const auto& a, const auto& b){
      return a.area >= b.area;  // larger is younger
    };
  std::sort(mdk.mens.begin(),   mdk.mens.end(),   howtosort);
  std::sort(mdk.dos.begin(),    mdk.dos.end(),    howtosort);
  std::sort(mdk.kotes.begin(),  mdk.kotes.end(),  howtosort);

  return {
    OpponentBehavior::STABLE_NEAR,
    !mdk.mens.empty() ? OpponentPart(mdk.mens[0].coord ) : boost::none,
    !mdk.dos.empty()  ? OpponentPart(mdk.dos[0].coord  ) : boost::none,
    !mdk.kotes.empty()? OpponentPart(mdk.kotes[0].coord) : boost::none,
  };
}
