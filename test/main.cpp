#include <cstdio>
#include <deque>
#include <chrono>
#include <iostream>
#include "../include/OpponentUnit.hpp"

void show_fps() {
  constexpr int range = 2;
  static std::deque<std::chrono::system_clock::time_point> cap;
  const auto now = std::chrono::system_clock::now();
  cap.push_back(now);
  while(cap.front() <= now-std::chrono::seconds(range)) {
    cap.pop_front();
  }
  std::cout << "FPS = " << 1.0*cap.size()/range << std::endl;
}

int main() {
  constexpr bool visualize = true;
  OpponentUnit o(30, visualize);

  while (true) {
    const auto s_ = o.survey(std::chrono::milliseconds(30));

#ifdef EVAL_PREDICTION
    continue;
#endif
    if(!s_) continue;   // frame is not yet ready
    show_fps();
    const auto s = s_.value();
    if(s.men)  { s.men.value().print("men  ");} else {std::cout << "men  none" << std::endl;}
    if(s.dou)  { s.dou.value().print("dou  ");} else {std::cout << "dou  none" << std::endl;}
    if(s.kote) {s.kote.value().print("kote ");} else {std::cout << "kote none" << std::endl;}
  }

  return EXIT_SUCCESS;
}
