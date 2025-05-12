#ifndef SKREDE_RSI_SPINLOCK_H
#define SKREDE_RSI_SPINLOCK_H

#include <atomic>

namespace skrede::rsi {

class Spinlock
{
public:
    Spinlock();

    void lock();

    void unlock();

private:
    std::atomic_flag m_lock;
};

}

#endif
