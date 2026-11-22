// SPDX-License-Identifier: MIT
// Linux-only: exercise the POSIX tracker IPC (shared memory + robust mutex)
// end to end without any devices. A writer publishes a fresh chest-role pose;
// a reader in a separate process must read it back and agree on freshness.
#include "vdxr_tracker_ipc.h"
#include <sys/wait.h>
#include <unistd.h>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

int main() {
    // Single-provider guard across processes: a child that tries to acquire the
    // same provider lock must fail (flock is per open file description, so the
    // check has to happen in a separate process).
    external_trackers::OwnerLock writerLock;
    if (!writerLock.acquire()) { std::fprintf(stderr, "provider lock acquire failed\n"); return 1; }
    {
        const pid_t guard = fork();
        if (guard == 0) {
            external_trackers::OwnerLock second;
            _exit(second.acquire() ? 1 : 0);
        }
        int guardStatus = 0; waitpid(guard, &guardStatus, 0);
        if (guardStatus != 0) { std::fprintf(stderr, "duplicate provider lock acquired\n"); return 1; }
    }

    external_trackers::Writer writer;
    if (!writer.create()) { std::fprintf(stderr, "writer create failed\n"); return 1; }

    const pid_t child = fork();
    if (child == 0) { _exit(execlp("true", "true", nullptr) < 0 ? 1 : 0); }
    int status = 0; waitpid(child, &status, 0);

    external_trackers::State published{};
    published.frequency = external_trackers::Frequency;
    published.samples[0].configured = 1;
    published.samples[0].qpc = external_trackers::now();
    for (int i = 0; i < 3; ++i) published.samples[0].position[i] = float(i) + .5f;
    const float orientation[] = {0, 0, 0, 1};
    for (int i = 0; i < 4; ++i) published.samples[0].orientation[i] = orientation[i];
    if (!writer.publish(published)) { std::fprintf(stderr, "publish failed\n"); return 1; }

    external_trackers::Reader reader;
    external_trackers::State received{};
    bool read = false;
    for (int attempt = 0; attempt < 50 && !read; ++attempt) {
        read = reader.read(received);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!read) { std::fprintf(stderr, "reader could not read the published state\n"); return 1; }
    if (received.magic != external_trackers::Magic || received.version != 1 || received.bytes != sizeof(received)) {
        std::fprintf(stderr, "ABI mismatch\n"); return 1;
    }
    if (received.frequency != external_trackers::Frequency) { std::fprintf(stderr, "frequency mismatch\n"); return 1; }
    if (!received.samples[0].configured || !external_trackers::fresh(received.samples[0], external_trackers::now(), received.frequency)) {
        std::fprintf(stderr, "published sample is not fresh\n"); return 1;
    }
    if (std::abs(received.samples[0].position[1] - 1.5f) > 1e-6) { std::fprintf(stderr, "position corrupted\n"); return 1; }

    // A stale timestamp must be reported as not fresh.
    external_trackers::State stale = published;
    stale.samples[0].qpc = external_trackers::now() - external_trackers::Frequency; // one second old
    writer.publish(stale);
    read = false;
    for (int attempt = 0; attempt < 50 && !read; ++attempt) {
        external_trackers::State probe{};
        read = reader.read(probe) && !external_trackers::fresh(probe.samples[0], external_trackers::now(), probe.frequency);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!read) { std::fprintf(stderr, "stale sample reported fresh\n"); return 1; }

    std::printf("Passed POSIX tracker IPC: shared state, robust lock, freshness window, provider lock\n");
    return 0;
}
