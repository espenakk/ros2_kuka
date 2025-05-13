#ifndef RSI_SPINLOCK_H
#define RSI_SPINLOCK_H

#include <atomic>

namespace rsi {

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
