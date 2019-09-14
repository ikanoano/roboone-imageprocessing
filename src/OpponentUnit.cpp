#include <algorithm>
#include "OpponentUnit.hpp"
#include "EigenUtil.hpp"
#include <iostream>

void OpponentUnit::startCamera() {} // To be implemented
void OpponentUnit::stopCamera() {}  // To be implemented
boost::optional<OpponentUnit::OpponentModel> OpponentUnit::survey(
    std::chrono::milliseconds predict_after) {
  const auto now = std::chrono::system_clock::now();
  auto mdk_ = m.get_men_do_kote();
  if(!mdk_) return boost::none;   // frame is not yet ready
  Mikagiri::men_do_kote_t mdk = mdk_.value();

#ifdef DEBUG_PREDICT
  std::cout <<
    "latency=" <<
    std::chrono::duration_cast<std::chrono::milliseconds>(now-mdk.frame_time_stamp).count() <<
    std::endl;
#endif

  // pick the largest target
  constexpr auto howtosort =
    [](const auto& a, const auto& b){
      return a.area >= b.area;    // larger is younger
    };
  std::sort(mdk.mens.begin(),   mdk.mens.end(),   howtosort);
  std::sort(mdk.dos.begin(),    mdk.dos.end(),    howtosort);
  std::sort(mdk.kotes.begin(),  mdk.kotes.end(),  howtosort);

  // add new mdk to history
  hist_mdk.push_back(mdk);

  // purge obsoleted history
  constexpr auto obsolete_th  = std::chrono::milliseconds(200);
  const     auto due          = mdk.frame_time_stamp - obsolete_th;
  while(hist_mdk.front().frame_time_stamp < due) {
    hist_mdk.pop_front();
    assert(!hist_mdk.empty() && "at least the last mdk is not obsolete");
  }

  // ROBOKEN's Prophet
  const auto predict_at = now + predict_after;
  const auto predict = [&](const auto &pick) -> OpponentPart {
    typedef std::chrono::duration<double, std::milli> double_ms;
    std::vector<double> t, x, y, z;
    // extract existing men/dou/kote history
    for (auto&& mdk: hist_mdk) {
      const auto *datotsu_part = pick(mdk);
      if(!datotsu_part) continue;   // not captured this time
      const std::array<float,3> &target = datotsu_part->coord;
      // shift t around t=0 to process acculate curve fitting
      const double shifted = std::chrono::duration_cast<double_ms>(mdk.frame_time_stamp - now + obsolete_th/2).count();
      // register recent captured position & time
      t.push_back(shifted);
      x.push_back(target[0]);
      y.push_back(target[1]);
      z.push_back(target[2]);
    }
    switch (t.size()) {
      case 0: return boost::none;   // not captured
      case 1: return {{(float)x[0], (float)y[0], (float)z[0]}}; // return the last position as we don't know except it
      default: // predict position using recent knowledge
        // polynomial curve fitting: get t-x, t-y and t-z curve
        const auto
          xcurve = EigenUtil::PolyFit(t, x, std::min(t.size()-1, (size_t)5)),
          ycurve = EigenUtil::PolyFit(t, y, std::min(t.size()-1, (size_t)5)),
          zcurve = EigenUtil::PolyFit(t, z, std::min(t.size()-1, (size_t)5));

        // solve with predict_at
        const double shifted = std::chrono::duration_cast<double_ms>(predict_at - now + obsolete_th/2).count();
#ifdef DEBUG_PREDICT
        std::cout <<
          "last x=" <<
          x.back() <<
          "\tpredict x=" <<
          boost::math::tools::evaluate_polynomial(xcurve.data().data(), shifted, xcurve.size()) <<
          std::endl;
#endif
        return {{
          (float)boost::math::tools::evaluate_polynomial(xcurve.data().data(), shifted, xcurve.size()),
          (float)boost::math::tools::evaluate_polynomial(ycurve.data().data(), shifted, ycurve.size()),
          (float)boost::math::tools::evaluate_polynomial(zcurve.data().data(), shifted, zcurve.size())
        }};
        break;
    }
  };

  const std::function<const Mikagiri::target_cand_t*(Mikagiri::men_do_kote_t&)>
    pick_men  = [](const auto &a) {return a.mens.empty()  ? nullptr : &a.mens[0];},
    pick_dou  = [](const auto &a) {return a.dos.empty()   ? nullptr : &a.dos[0];},
    pick_kote = [](const auto &a) {return a.kotes.empty() ? nullptr : &a.kotes[0];};

  // predict men/dou/kote
  OpponentModel om;
  om.men  = predict(pick_men);
  om.dou  = predict(pick_dou);
  om.kote = predict(pick_kote);

  // TODO add some process to get OpponentBehavior ...
  om.behavior = OpponentBehavior::STABLE_NEAR;

  return om;
}
