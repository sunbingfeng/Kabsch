/**
 * Author: Bill(cocobill1987ATgmail.com)
 * Description: Demo
 * License: see the LICENSE.txt file
 */
#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::high_resolution_clock::now();
    }

    double toc()
    {
        end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
};
