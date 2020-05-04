#ifndef OPPONENT_UNIT
#define OPPONENT_UNIT

#include <deque>
#include <chrono>
#include <iostream>
#include <boost/optional.hpp>
#include <boost/math/tools/polynomial.hpp>
#include "Mikagiri.hpp"
#ifdef USE_CAMERA
  #include "Mikiri.hpp"
#endif

class OpponentUnit {
public:
  OpponentUnit(int fps=30, bool visualize=true) : m(fps,visualize) {}

  enum class OpponentBehavior {
    STABLE_FAR_OUT_OF_RANGE,
    STABLE_FAR_IN_RANGE,
    STABLE_NEAR,
    APPROACHING_VERTICAL,
    APPROACHING_HORIZONTAL,
    EVACUATING
  };
  typedef std::chrono::system_clock::time_point time_stamp_t;
  struct OpponentPart {
    std::array<float, 3>  last;
    time_stamp_t          last_at;
    std::array<boost::math::tools::polynomial<double>, 3>
                          curve;
    std::chrono::time_point<std::chrono::system_clock>
                          curve_0;

    inline void print(const char* pre) const {
      std::cout
        << pre
        << " last @ "
        << std::chrono::system_clock::to_time_t(last_at)
        << " = (" << last[0] << ", " << last[1] << ", " << last[2] << ")"
        //<< "\tpred @ "
        //<< std::chrono::system_clock::to_time_t(pred_at)
        //<< " = (" << pred[0] << ", " << pred[1] << ", " << pred[2] << ")"
        << std::endl;
    }

    std::array<float, 3> predict(time_stamp_t predict_at) const {
      typedef std::chrono::duration<double, std::milli> double_ms;
      // solve with predict_at
      const double shifted = std::chrono::duration_cast<double_ms>(predict_at - curve_0).count();
      return {{
        (float)boost::math::tools::evaluate_polynomial(curve[0].data().data(), shifted, curve[0].size()),
        (float)boost::math::tools::evaluate_polynomial(curve[1].data().data(), shifted, curve[1].size()),
        (float)boost::math::tools::evaluate_polynomial(curve[2].data().data(), shifted, curve[2].size())
      }};
    }

    template<typename T>
    inline T last_xyz() const { return T{.x=last[0], .y=last[1], .z=last[2]}; }
    template<typename T>
    inline T pred_xyz(time_stamp_t predict_at) const {
      const auto pred = predict(predict_at);
      return T{.x=pred[0], .y=pred[1], .z=pred[2]};
    }
  };
  struct OpponentModel {
    OpponentBehavior              behavior;
    boost::optional<OpponentPart> men;
    boost::optional<OpponentPart> dou;
    boost::optional<OpponentPart> kote;
  };

  void startCamera();
  void stopCamera();
  boost::optional<OpponentModel> survey();
private:
#ifdef USE_CAMERA
  Mikiri    m;
#else
  Mikagiri  m;
#endif
  std::deque<Mikagiri::men_do_kote_t>  hist_mdk;
};

#endif
