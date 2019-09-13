#include <cstdio>
#include <deque>
#include <chrono>
#include <iostream>
#include "../src/OpponentUnit.hpp"

void printOpponent(const char* pre, const OpponentUnit::OpponentPart &op) {
  if(!op) {
    std::cout << pre << " none" << std::endl;
    return;
  }
  std::cout
    << pre << ": ("
    << op.value()[0] << ", "
    << op.value()[1] << ", "
    << op.value()[2] << ")"
    << std::endl;
}

void show_fps() {
  constexpr int range = 2;
  static std::deque<std::chrono::system_clock::time_point> cap;
  const auto now = std::chrono::system_clock::now();
  cap.push_back(now);
  while(cap.front() < now-std::chrono::seconds(range)) {
    cap.pop_front();
  }
  std::cout << "FPS = " << 1.0*cap.size()/range << std::endl;
}

int main(int argc, char * argv[]) {

  OpponentUnit o;

  while (true) {
    const auto s_ = o.survey();
    if(!s_) continue;   // frame is not yet ready
    show_fps();
    const auto s = s_.value();
    printOpponent("men ", s.men);
    printOpponent("dou ", s.dou);
    printOpponent("kote", s.kote);
  }

  return EXIT_SUCCESS;
}
