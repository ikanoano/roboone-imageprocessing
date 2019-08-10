#include <algorithm>
#include "opponent_unit.hpp"
#include "men_do_kote.hpp"

OpponentUnit::OpponentUnit(int fps) : m(fps) {
  cv::namedWindow(visual_window,  cv::WINDOW_AUTOSIZE);
}

void OpponentUnit::startCamera() {} // To be implemented
void OpponentUnit::stopCamera() {}  // To be implemented
OpponentUnit::OpponentModel OpponentUnit::survey() {
  cv::Mat visual_mat;
  auto mdk = m.get_men_do_kote(visual_mat);
  cv::imshow(visual_window, visual_mat);

  // pick largest target
  constexpr auto howtosort =
    [](const auto& a, const auto& b){
      return a.area >= b.area;  // larger is younger
    };
  std::sort(mdk.mens.begin(),   mdk.mens.end(),   howtosort);
  std::sort(mdk.dos.begin(),    mdk.dos.end(),    howtosort);
  std::sort(mdk.kotes.begin(),  mdk.kotes.end(),  howtosort);

  return {
    mdk.mens.empty()  ? OpponentPart(mdk.mens[0].coord ) : std::nullopt,
    mdk.dos.empty()   ? OpponentPart(mdk.dos[0].coord  ) : std::nullopt,
    mdk.kotes.empty() ? OpponentPart(mdk.kotes[0].coord) : std::nullopt,
  };
}
