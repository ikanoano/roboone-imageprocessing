#include <random>
#include <cmath>
#include <random>
#include <iostream>
#include "Mikagiri.hpp"

boost::optional<Mikagiri::men_do_kote_t> Mikagiri::get_men_do_kote() {
  static  auto last = std::chrono::system_clock::now();
  const   auto now  = std::chrono::system_clock::now();
  assert((now - last < std::chrono::milliseconds(200)) && "too late to call");
  if(now-last < interval) return boost::none;
  const   auto time_stamp = last + interval - std::chrono::milliseconds(100);
  assert(time_stamp <= now);
  last = now;

  // sometimes return boost::none as if they are not captured
  int  rnd   = rand();
  bool menex = ((rnd>> 0) & 0xFF) < 0xC0;
  bool douex = ((rnd>> 8) & 0xFF) < 0xC0;
  bool koteex= ((rnd>>16) & 0xFF) < 0xC0;

  const double t = time_stamp.time_since_epoch()/std::chrono::milliseconds(1) / 1000.0;
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  std::normal_distribution<float> dist(0.0, 0.005);
  decltype(target_cand_t::coord)
    m = {
      (float)(0.40 + 0.30*std::sin(3.00*t)) + dist(engine),
      (float)(0.00 + 0.05*std::sin(0.10*t)) + dist(engine),
      (float)(0.20 + 0.05*std::sin(0.33*t)) + dist(engine)
    },
    d = {
      (float)(0.45 + 0.30*std::sin(3.00*t)) + dist(engine),
      (float)(0.00 + 0.04*std::sin(0.10*t)) + dist(engine),
      (float)(0.15 + 0.06*std::sin(0.33*t)) + dist(engine)
    },
    k = {
      (float)(0.35 + 0.40*std::sin(3.00*t)) + dist(engine),
      (float)(0.00 + 0.10*std::sin(0.10*t)) + dist(engine),
      (float)(0.18 + 0.10*std::sin(0.33*t)) + dist(engine)
    };
  std::vector<target_cand_t>  mens, dos, kotes;
  if(menex)    mens.push_back({m, 10});
  if(douex)     dos.push_back({d, 10});
  if(koteex)  kotes.push_back({k, 10});

  return men_do_kote_t{time_stamp, mens, dos, kotes};
}
