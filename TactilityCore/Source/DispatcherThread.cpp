#include "Tactility/DispatcherThread.h"

#ifdef __EMSCRIPTEN__
#include <algorithm>
#include <vector>
#endif

namespace tt {

#ifdef __EMSCRIPTEN__
namespace {
    std::vector<DispatcherThread*> wasmDispatcherThreads;
}
#endif

DispatcherThread::DispatcherThread(const std::string& threadName, size_t threadStackSize) {
#ifndef __EMSCRIPTEN__
    thread = std::make_unique<Thread>(
        threadName,
        threadStackSize,
        [this] {
            return threadMain();
        }
    );
#else
    (void)threadName;
    (void)threadStackSize;
    wasmDispatcherThreads.push_back(this);
#endif
}

DispatcherThread::~DispatcherThread() {
#ifndef __EMSCRIPTEN__
    if (thread && thread->getState() != Thread::State::Stopped) {
        stop();
    }
#else
    auto it = std::find(wasmDispatcherThreads.begin(), wasmDispatcherThreads.end(), this);
    if (it != wasmDispatcherThreads.end()) {
        wasmDispatcherThreads.erase(it);
    }
#endif
}

int32_t DispatcherThread::threadMain() {
    do {
        /**
         * If this value is too high (e.g. 1 second) then the dispatcher destroys too slowly when the simulator exits.
         * This causes the problems with other services doing an update (e.g. Statusbar) and calling into destroyed mutex in the global scope.
         */
        dispatcher.consume(100 / portTICK_PERIOD_MS);
    } while (!interruptThread);

    return 0;
}

bool DispatcherThread::dispatch(Dispatcher::Function function, TickType_t timeout) {
#ifdef __EMSCRIPTEN__
    return dispatcher.dispatch(std::move(function), timeout);
#else
    return dispatcher.dispatch(std::move(function), timeout);
#endif
}

void DispatcherThread::start() {
#ifdef __EMSCRIPTEN__
    interruptThread = false;
#else
    interruptThread = false;
    thread->start();
#endif
}

void DispatcherThread::stop() {
    interruptThread = true;
#ifndef __EMSCRIPTEN__
    thread->join();
#endif
}

#ifdef __EMSCRIPTEN__
void DispatcherThread::pumpAll() {
    for (auto* instance : wasmDispatcherThreads) {
        if (instance != nullptr && !instance->interruptThread) {
            instance->dispatcher.consume(0);
        }
    }
}
#endif

}