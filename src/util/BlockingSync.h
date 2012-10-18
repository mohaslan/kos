/******************************************************************************
    Copyright 2012 Martin Karsten

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/
#ifndef _BlockingSync_h_
#define _BlockingSync_h_ 1

#include "util/EmbeddedQueue.h"
#include "util/SpinLock.h"
#include "mach/Processor.h"
#include "kern/globals.h"
#include "kern/Kernel.h"
#include "kern/Scheduler.h"
#include "kern/Thread.h"

class BlockingSync {
  EmbeddedQueue<Thread> threadQueue;
protected:
  SpinLock lk;
  ~BlockingSync() {}
  void suspend() {
    threadQueue.push(Processor::getThread());
    kernelScheduler.suspend(lk);
  }
  Thread* resume() {
    Thread* t = threadQueue.front();
    threadQueue.pop();
    kernelScheduler.start(*t);
    return t;
  }
  bool waiting() {
    return !threadQueue.empty();
  }
};

class Semaphore : public BlockingSync {
  int counter;
public:
  Semaphore(int c = 0) : counter(c) {}
  void operator delete(ptr_t ptr) { globaldelete(ptr, sizeof(Semaphore)); }
    
  void P() {
    LockObject<SpinLock> lo(lk);
    if likely(counter < 1) suspend();
    else counter -= 1;
  }
  bool tryP() {
    LockObject<SpinLock> lo(lk);
    if likely(counter < 1) return false;
    counter -= 1;
    return true;
  }
  void V() {
    lk.acquire();
    if likely(waiting()) {
      resume(); // pass closed lock to thread waiting on V()
    } else {
      counter += 1;
      lk.release();
    }
  }
};

class Mutex : public BlockingSync {
protected:
  Thread* owner;
public:
  Mutex() : owner(nullptr) {}
  void operator delete(ptr_t ptr) { globaldelete(ptr, sizeof(Mutex)); }
  void acquire() {
    LockObject<SpinLock> lo(lk);
    if (owner) suspend();
    else owner = Processor::getThread();
  }
  void release() {
    lk.acquire();
    KASSERT(owner == Processor::getThread(), "attempt to release lock by non-owner");
    if likely(waiting()) {
      owner = resume();
    } else {
      owner = nullptr;
      lk.release();
    }
  }
};

#endif /* _BlockingSync_h_ */
