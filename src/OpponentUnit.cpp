#include <algorithm>
#include "OpponentUnit.hpp"

void OpponentUnit::startCamera() {} // To be implemented
void OpponentUnit::stopCamera() {}  // To be implemented
boost::optional<OpponentUnit::OpponentModel> OpponentUnit::survey() {
  auto mdk_ = m.get_men_do_kote();
  if(!mdk_) return boost::none;  // frame is not yet ready
  auto mdk = mdk_.value();

  // pick largest target
  constexpr auto howtosort =
    [](const auto& a, const auto& b){
      return a.area >= b.area;  // larger is younger
    };
  std::sort(mdk.mens.begin(),   mdk.mens.end(),   howtosort);
  std::sort(mdk.dos.begin(),    mdk.dos.end(),    howtosort);
  std::sort(mdk.kotes.begin(),  mdk.kotes.end(),  howtosort);

  return OpponentModel {
    OpponentBehavior::STABLE_NEAR,
    !mdk.mens.empty() ? OpponentPart(mdk.mens[0].coord ) : boost::none,
    !mdk.dos.empty()  ? OpponentPart(mdk.dos[0].coord  ) : boost::none,
    !mdk.kotes.empty()? OpponentPart(mdk.kotes[0].coord) : boost::none,
  };
}
