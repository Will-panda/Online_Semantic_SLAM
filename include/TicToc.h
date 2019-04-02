//
// Created by liwenqiang on 3/31/19.
//

#ifndef OS_SLAM_TICTOC_H
#define OS_SLAM_TICTOC_H


//#include <ctime>
//#include <cstdlib>
#include <chrono>

class TicToc{
public:
  TicToc(){ Tic();  }
  void Tic() {start= std::chrono::steady_clock::now();}
  double Toc(){
      end = std::chrono::steady_clock::now();
      std::chrono::duration<double> costSeconds = end - start;
      return costSeconds.count() * 1000;
  }

private:
  std::chrono::steady_clock::time_point start;
  std::chrono::steady_clock::time_point end;
};
#endif //OS_SLAM_TICTOC_H
