#ifndef TIMER_HPP_INCLUDED
#define TIMER_HPP_INCLUDED
#include <iostream>
#include <chrono>
#include <string>

class Timer
{
public:
    using Seconds = double;
    void start()
    {
        m_start = std::chrono::high_resolution_clock::now();
    }
    void stop()
    {
        m_stop = std::chrono::high_resolution_clock::now();
    }
    Seconds duration()
    {
        std::chrono::duration<Seconds> dur(m_stop - m_start);
        return dur.count();
    }
public:
    using Timepoint = decltype(std::chrono::high_resolution_clock::now());
    Timepoint m_start;
    Timepoint m_stop;
};

#endif // TIMER_HPP_INCLUDED

