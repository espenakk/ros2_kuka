#include "skrede/rsi/spinlock.h"

using namespace skrede::rsi;

Spinlock::Spinlock()
    : m_lock{}
{
}

void Spinlock::lock()
{
    while(m_lock.test_and_set(std::memory_order_acquire));
}

void Spinlock::unlock()
{
    m_lock.clear(std::memory_order_release);
}
