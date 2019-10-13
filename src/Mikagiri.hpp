#ifndef MIKAGIRI_H
#define MIKAGIRI_H

#include <boost/optional.hpp>
#include <vector>
#include <chrono>

class Mikagiri {
public:
  struct target_cand_t {
    std::array<float, 3>  coord;
    int                   area;
  };
  typedef std::chrono::system_clock::time_point time_stamp_t;
  struct men_do_kote_t {
    time_stamp_t                frame_time_stamp;
    std::vector<target_cand_t>  mens;
    std::vector<target_cand_t>  dos;
    std::vector<target_cand_t>  kotes;
  };

  const int   FPS;
  const bool  visualize;

  Mikagiri() = delete;
  Mikagiri(int fps, bool visualize) :
    FPS(fps),
    visualize(visualize),
    interval(std::chrono::milliseconds(1000/fps)) {}

  virtual                         ~Mikagiri() {};
  virtual void                    start() {};
  virtual void                    stop()  {};
  virtual boost::optional<men_do_kote_t>
                                  get_men_do_kote();

protected:
  const std::chrono::milliseconds interval;
};

#endif
