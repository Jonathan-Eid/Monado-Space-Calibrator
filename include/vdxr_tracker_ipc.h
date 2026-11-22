// SPDX-License-Identifier: MIT
#pragma once

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <pthread.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

// Private, versioned IPC for the experimental SteamVR-free provider. Poses are
// in the current Quest STAGE space: metres, right handed, quaternion XYZW.
//
// Windows keeps named shared memory plus a named mutex; POSIX uses named
// POSIX shared memory (/dev/shm) plus a process-shared robust mutex stored in
// a companion lock segment. Timestamps are QPC ticks on Windows and
// CLOCK_MONOTONIC nanoseconds on POSIX (Frequency below).
namespace external_trackers {
inline constexpr uint32_t Magic = 0x56585431;
inline constexpr std::array<const char*, 14> Roles = {
    "chest", "waist", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_knee", "right_knee", "left_ankle", "right_ankle",
    "left_foot", "right_foot"};
#pragma pack(push, 8)
struct Sample {
    int64_t qpc{};
    float position[3]{};
    float orientation[4]{0, 0, 0, 1};
    uint32_t configured{};
};
struct State {
    uint32_t magic{Magic};
    uint32_t version{1};
    uint32_t bytes{sizeof(State)};
    uint32_t reserved{};
    int64_t frequency{};
    Sample samples[Roles.size()]{};
};
#pragma pack(pop)
static_assert(sizeof(Sample) == 40 && sizeof(State) == 584, "IPC ABI changed");

inline bool fresh(const Sample& sample, int64_t now, int64_t frequency) {
    if (!sample.configured || sample.qpc <= 0 || frequency <= 0 || now < sample.qpc ||
        now - sample.qpc > frequency / 10) {
        return false;
    }
    double norm = 0;
    for (float v : sample.position) if (!std::isfinite(v)) return false;
    for (float v : sample.orientation) {
        if (!std::isfinite(v)) return false;
        norm += double(v) * v;
    }
    return std::abs(norm - 1) < 0.01;
}

#ifdef _WIN32
#include <windows.h>
inline constexpr wchar_t MappingName[] = L"Local\\VDXR.Libsurvive.v1";
inline constexpr wchar_t MutexName[] = L"Local\\VDXR.Libsurvive.Lock.v1";

class Reader {
public:
    ~Reader() {
        if (m_view) UnmapViewOfFile(m_view);
        if (m_mapping) CloseHandle(m_mapping);
        if (m_mutex) CloseHandle(m_mutex);
    }
    bool read(State& state) const {
        std::lock_guard<std::mutex> guard(m_localMutex);
        // Retry opening if the provider was not running yet.
        if (!m_mutex) m_mutex = OpenMutexW(SYNCHRONIZE | MUTEX_MODIFY_STATE, FALSE, MutexName);
        if (!m_mapping) m_mapping = OpenFileMappingW(FILE_MAP_READ, FALSE, MappingName);
        if (!m_view && m_mapping) m_view = MapViewOfFile(m_mapping, FILE_MAP_READ, 0, 0, sizeof(State));
        if (!m_mutex || !m_view) return false;
        const auto result = WaitForSingleObject(m_mutex, 0);
        if (result == WAIT_ABANDONED) {
            ReleaseMutex(m_mutex);
            return false;
        }
        if (result != WAIT_OBJECT_0) return false; // Never wait on the rendering thread.
        std::memcpy(&state, m_view, sizeof(state));
        ReleaseMutex(m_mutex);
        LARGE_INTEGER frequency;
        QueryPerformanceFrequency(&frequency);
        return state.magic == Magic && state.version == 1 && state.bytes == sizeof(State) &&
               state.frequency == frequency.QuadPart;
    }
private:
    mutable std::mutex m_localMutex;
    mutable HANDLE m_mapping{}, m_mutex{};
    mutable void* m_view{};
};
#else
inline constexpr char MappingName[] = "/VDXR.Libsurvive.v1";      // shm_open data segment
inline constexpr char LockName[] = "/VDXR.Libsurvive.Lock.v1";   // shm_open mutex segment
inline constexpr char OwnerFile[] = "VDXR.Libsurvive.Writer.v1"; // single-provider flock
// CLOCK_MONOTONIC ticks per second; State::frequency must equal this.
inline constexpr int64_t Frequency = 1000000000;
inline int64_t now() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec) * Frequency + ts.tv_nsec;
}

// Process-shared robust mutex living in the lock segment; NULL until mapped.
class Reader {
public:
    ~Reader() {
        if (m_view) munmap(m_view, sizeof(State));
        if (m_lockView) munmap(m_lockView, sizeof(pthread_mutex_t));
        if (m_data >= 0) close(m_data);
        if (m_lock >= 0) close(m_lock);
    }
    bool read(State& state) const {
        std::lock_guard<std::mutex> guard(m_localMutex);
        // Retry opening if the provider was not running yet.
        if (m_data < 0) m_data = shm_open(MappingName, O_RDONLY, 0);
        if (m_lock < 0) m_lock = shm_open(LockName, O_RDWR, 0);
        if (!m_view && m_data >= 0) {
            struct stat info{};
            if (fstat(m_data, &info) == 0 && info.st_size >= off_t(sizeof(State)))
                m_view = mmap(nullptr, sizeof(State), PROT_READ, MAP_SHARED, m_data, 0);
            else { close(m_data); m_data = -1; } // not fully created yet; retry later
        }
        if (!m_lockView && m_lock >= 0) {
            struct stat info{};
            if (fstat(m_lock, &info) == 0 && info.st_size >= off_t(sizeof(pthread_mutex_t)))
                m_lockView = mmap(nullptr, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, m_lock, 0);
            else { close(m_lock); m_lock = -1; }
        }
        if (!m_view || !m_lockView) return false;
        auto* mutex = static_cast<pthread_mutex_t*>(m_lockView);
        const int result = pthread_mutex_trylock(mutex);
        if (result == EOWNERDEAD) { // provider died while holding the lock
            pthread_mutex_consistent(mutex);
            pthread_mutex_unlock(mutex);
            return false;
        }
        if (result != 0) return false; // Never wait on the rendering thread.
        std::memcpy(&state, m_view, sizeof(state));
        pthread_mutex_unlock(mutex);
        return state.magic == Magic && state.version == 1 && state.bytes == sizeof(State) &&
               state.frequency == Frequency;
    }
private:
    mutable std::mutex m_localMutex;
    mutable int m_data{-1}, m_lock{-1};
    mutable void* m_view{}, *m_lockView{};
};

// Publisher side (tracking service). create() recycles any stale segments;
// the caller is expected to hold the single-provider flock below first.
class Writer {
public:
    ~Writer() {
        if (m_view) munmap(m_view, sizeof(State));
        if (m_lockView) munmap(m_lockView, sizeof(pthread_mutex_t));
        if (m_data >= 0) { close(m_data); shm_unlink(MappingName); }
        if (m_lock >= 0) { close(m_lock); shm_unlink(LockName); }
    }
    bool create() {
        shm_unlink(MappingName);
        shm_unlink(LockName);
        m_data = shm_open(MappingName, O_RDWR | O_CREAT | O_EXCL, 0600);
        m_lock = shm_open(LockName, O_RDWR | O_CREAT | O_EXCL, 0600);
        if (m_data < 0 || m_lock < 0 ||
            ftruncate(m_data, sizeof(State)) != 0 ||
            ftruncate(m_lock, sizeof(pthread_mutex_t)) != 0) return false;
        m_view = mmap(nullptr, sizeof(State), PROT_READ | PROT_WRITE, MAP_SHARED, m_data, 0);
        m_lockView = mmap(nullptr, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, m_lock, 0);
        if (m_view == MAP_FAILED || m_lockView == MAP_FAILED) { m_view = m_lockView = nullptr; return false; }
        auto* mutex = static_cast<pthread_mutex_t*>(m_lockView);
        pthread_mutexattr_t attributes;
        if (pthread_mutexattr_init(&attributes) != 0 ||
            pthread_mutexattr_setpshared(&attributes, PTHREAD_PROCESS_SHARED) != 0 ||
            pthread_mutexattr_setrobust(&attributes, PTHREAD_MUTEX_ROBUST) != 0 ||
            pthread_mutex_init(mutex, &attributes) != 0) {
            pthread_mutexattr_destroy(&attributes);
            return false;
        }
        pthread_mutexattr_destroy(&attributes);
        State state{};
        std::memcpy(m_view, &state, sizeof(state));
        return true;
    }
    bool publish(const State& state) {
        if (!m_view || !m_lockView) return false;
        auto* mutex = static_cast<pthread_mutex_t*>(m_lockView);
        const int result = pthread_mutex_trylock(mutex);
        if (result == EOWNERDEAD) pthread_mutex_consistent(mutex); // recover and overwrite anyway
        else if (result != 0) return false;
        std::memcpy(m_view, &state, sizeof(state));
        pthread_mutex_unlock(mutex);
        return true;
    }
private:
    int m_data{-1}, m_lock{-1};
    void* m_view{}, *m_lockView{};
};

// Single-provider guard equivalent to the Windows Local\VDXR.Libsurvive.Writer.v1
// named mutex: an exclusive flock on $XDG_RUNTIME_DIR/VDXR.Libsurvive.Writer.v1.
class OwnerLock {
public:
    ~OwnerLock() { if (m_fd >= 0) close(m_fd); }
    bool acquire() {
        const char* directory = getenv("XDG_RUNTIME_DIR");
        if (!directory || !*directory) directory = "/tmp";
        const std::string path = std::string(directory) + "/" + OwnerFile;
        m_fd = open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
        if (m_fd < 0) return false;
        return flock(m_fd, LOCK_EX | LOCK_NB) == 0; // released automatically on process death
    }
private:
    int m_fd{-1};
};
#endif
} // namespace external_trackers
