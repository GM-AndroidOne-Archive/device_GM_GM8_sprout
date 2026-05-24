/*
 * AgressivePowerHAL (msm8937)
 * Ported from shamrock aggressive sysfs-boost approach.
 *  - Use interactive governor boost + sched_boost + (optional) hispeed_freq + KGSL min_pwrlevel
 *  - Timed boost with safe restore (generation counter).
 */

#define LOG_NIDEBUG 0
#define LOG_TAG "AgressivePowerHAL_msm8937 : "

#include <cutils/log.h>
#include <hardware/power.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "power-common.h"   // for HINT_HANDLED/HINT_NONE + profiles (if present in your tree)
#include "utils.h"          // some trees provide helpers; we only rely on macros if present

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

// --------------------------- paths (GM8 kernel) -------------------------------

static const char* kSchedBoostPath = "/proc/sys/kernel/sched_boost";

static const char* kCpu0BoostPath   = "/sys/devices/system/cpu/cpu0/cpufreq/interactive/boost";
static const char* kCpu4BoostPath   = "/sys/devices/system/cpu/cpu4/cpufreq/interactive/boost";
static const char* kCpu0HispeedPath = "/sys/devices/system/cpu/cpu0/cpufreq/interactive/hispeed_freq";
static const char* kCpu4HispeedPath = "/sys/devices/system/cpu/cpu4/cpufreq/interactive/hispeed_freq";

static const char* kCpu0MaxFreqPath = "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq";
static const char* kCpu4MaxFreqPath = "/sys/devices/system/cpu/cpu4/cpufreq/cpuinfo_max_freq";

static const char* kKgslBasePath    = "/sys/devices/soc/1c00000.qcom,kgsl-3d0/kgsl/kgsl-3d0";
static const char* kKgslMinPwrPath  = "/sys/devices/soc/1c00000.qcom,kgsl-3d0/kgsl/kgsl-3d0/min_pwrlevel";
static const char* kKgslDefPwrPath  = "/sys/devices/soc/1c00000.qcom,kgsl-3d0/kgsl/kgsl-3d0/default_pwrlevel";

// --------------------------- helpers -----------------------------------------

static int read_str(const char* path, char* out, size_t out_sz);
static int read_int(const char* path, int* out);
static int write_str_file(const char* path, const char* s);
static int write_int(const char* path, int v);

static int path_exists(const char* path) {
    return (path && access(path, F_OK) == 0);
}

static int should_disable_for_rc(int rc) {
    // Disable on hard errors: permission denied, no such file
    return (rc == -EACCES || rc == -ENOENT || rc == -EPERM);
}

// --------------------------- global state ------------------------------------

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static long long g_gen = 0;

static int g_disabled_sched = 0;
static int g_disabled_cpu = 0;
static int g_disabled_gpu = 0;

// cached defaults
static int g_def_sched_boost = -1;
static int g_def_cpu0_boost = -1;
static int g_def_cpu4_boost = -1;
static int g_def_cpu0_hispeed = -1;
static int g_def_cpu4_hispeed = -1;
static int g_def_gpu_min_pwr = -1;

// computed "max" for launch
static int g_cpu0_max = 1401000; // safe fallback from your dump
static int g_cpu4_max = 1094400; // safe fallback from your dump
static int g_gpu_default_pwr = -1;

static void cache_defaults_locked(void) {
    if (g_def_sched_boost < 0 && path_exists(kSchedBoostPath)) {
        int v;
        if (read_int(kSchedBoostPath, &v) == 0) g_def_sched_boost = v;
    }

    if (g_def_cpu0_boost < 0 && path_exists(kCpu0BoostPath)) {
        int v;
        if (read_int(kCpu0BoostPath, &v) == 0) g_def_cpu0_boost = v;
    }
    if (g_def_cpu4_boost < 0 && path_exists(kCpu4BoostPath)) {
        int v;
        if (read_int(kCpu4BoostPath, &v) == 0) g_def_cpu4_boost = v;
    }

    if (g_def_cpu0_hispeed < 0 && path_exists(kCpu0HispeedPath)) {
        int v;
        if (read_int(kCpu0HispeedPath, &v) == 0) g_def_cpu0_hispeed = v;
    }
    if (g_def_cpu4_hispeed < 0 && path_exists(kCpu4HispeedPath)) {
        int v;
        if (read_int(kCpu4HispeedPath, &v) == 0) g_def_cpu4_hispeed = v;
    }

    if (g_def_gpu_min_pwr < 0 && path_exists(kKgslMinPwrPath)) {
        int v;
        if (read_int(kKgslMinPwrPath, &v) == 0) g_def_gpu_min_pwr = v;
    }

    if (g_gpu_default_pwr < 0 && path_exists(kKgslDefPwrPath)) {
        int v;
        if (read_int(kKgslDefPwrPath, &v) == 0) g_gpu_default_pwr = v;
    }
}

static void restore_locked(void) {
    if (!g_disabled_sched && g_def_sched_boost >= 0 && path_exists(kSchedBoostPath)) {
        int rc = write_int(kSchedBoostPath, g_def_sched_boost);
        if (rc != 0 && should_disable_for_rc(rc)) g_disabled_sched = 1;
    }

    if (!g_disabled_cpu) {
        if (g_def_cpu0_boost >= 0 && path_exists(kCpu0BoostPath)) {
            int rc = write_int(kCpu0BoostPath, g_def_cpu0_boost);
            if (rc != 0 && should_disable_for_rc(rc)) g_disabled_cpu = 1;
        }
        if (!g_disabled_cpu && g_def_cpu4_boost >= 0 && path_exists(kCpu4BoostPath)) {
            int rc = write_int(kCpu4BoostPath, g_def_cpu4_boost);
            if (rc != 0 && should_disable_for_rc(rc)) g_disabled_cpu = 1;
        }
        if (!g_disabled_cpu && g_def_cpu0_hispeed >= 0 && path_exists(kCpu0HispeedPath)) {
            int rc = write_int(kCpu0HispeedPath, g_def_cpu0_hispeed);
            if (rc != 0 && should_disable_for_rc(rc)) g_disabled_cpu = 1;
        }
        if (!g_disabled_cpu && g_def_cpu4_hispeed >= 0 && path_exists(kCpu4HispeedPath)) {
            int rc = write_int(kCpu4HispeedPath, g_def_cpu4_hispeed);
            if (rc != 0 && should_disable_for_rc(rc)) g_disabled_cpu = 1;
        }
    }

    if (!g_disabled_gpu && g_def_gpu_min_pwr >= 0 && path_exists(kKgslMinPwrPath)) {
        int rc = write_int(kKgslMinPwrPath, g_def_gpu_min_pwr);
        if (rc != 0 && should_disable_for_rc(rc)) g_disabled_gpu = 1;
    }
}

static void restore_now(void) {
    pthread_mutex_lock(&g_lock);
    restore_locked();
    pthread_mutex_unlock(&g_lock);
}

typedef struct {
    long long gen;
    int delay_ms;
} restore_task_t;

static void* restore_thread(void* arg) {
    restore_task_t* t = (restore_task_t*)arg;
    if (!t) return NULL;

    // sleep
    if (t->delay_ms > 0) usleep((useconds_t)t->delay_ms * 1000);

    pthread_mutex_lock(&g_lock);
    if (t->gen == g_gen) {
        restore_locked();
    }
    pthread_mutex_unlock(&g_lock);

    free(t);
    return NULL;
}

static void schedule_restore_locked(int delay_ms, long long gen) {
    restore_task_t* t = (restore_task_t*)calloc(1, sizeof(*t));
    if (!t) return;
    t->gen = gen;
    t->delay_ms = delay_ms;

    pthread_t th;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    if (pthread_create(&th, &attr, restore_thread, t) != 0) {
        free(t);
    }
    pthread_attr_destroy(&attr);
}

static void apply_boost_timed(
        int set_sched_boost,        // -1 skip, else 0/1
        int set_cpu_boost,          // -1 skip, else 0/1 for both clusters
        int set_cpu0_hispeed,       // -1 skip, else freq
        int set_cpu4_hispeed,       // -1 skip, else freq
        int set_gpu_min_pwrlevel,   // -1 skip, else pwrlevel
        int duration_ms) {

    pthread_mutex_lock(&g_lock);
    cache_defaults_locked();

    g_gen++;
    long long mygen = g_gen;

    // Apply
    if (!g_disabled_sched && set_sched_boost >= 0 && path_exists(kSchedBoostPath)) {
        int rc = write_int(kSchedBoostPath, set_sched_boost);
        if (rc != 0 && should_disable_for_rc(rc)) g_disabled_sched = 1;
    }

    if (!g_disabled_cpu && set_cpu_boost >= 0) {
        if (path_exists(kCpu0BoostPath)) {
            int rc = write_int(kCpu0BoostPath, set_cpu_boost);
            if (rc != 0 && should_disable_for_rc(rc)) g_disabled_cpu = 1;
        }
        if (!g_disabled_cpu && path_exists(kCpu4BoostPath)) {
            int rc = write_int(kCpu4BoostPath, set_cpu_boost);
            if (rc != 0 && should_disable_for_rc(rc)) g_disabled_cpu = 1;
        }
    }

    if (!g_disabled_cpu && set_cpu0_hispeed >= 0 && path_exists(kCpu0HispeedPath)) {
        int rc = write_int(kCpu0HispeedPath, set_cpu0_hispeed);
        if (rc != 0 && should_disable_for_rc(rc)) g_disabled_cpu = 1;
    }
    if (!g_disabled_cpu && set_cpu4_hispeed >= 0 && path_exists(kCpu4HispeedPath)) {
        int rc = write_int(kCpu4HispeedPath, set_cpu4_hispeed);
        if (rc != 0 && should_disable_for_rc(rc)) g_disabled_cpu = 1;
    }

    if (!g_disabled_gpu && set_gpu_min_pwrlevel >= 0 && path_exists(kKgslMinPwrPath)) {
        int rc = write_int(kKgslMinPwrPath, set_gpu_min_pwrlevel);
        if (rc != 0 && should_disable_for_rc(rc)) g_disabled_gpu = 1;
    }

    // Timed restore
    if (duration_ms > 0) {
        schedule_restore_locked(duration_ms, mygen);
    }

    pthread_mutex_unlock(&g_lock);
}

// --------------------------- hint logic --------------------------------------

static void process_interaction_hint(void* data) {
    // data (ms) sometimes passed by framework; clamp
    int duration = 150;
    if (data) {
        int d = *((int*)data);
        if (d > 0 && d < 5000) duration = d;
    }
    if (duration < 80) duration = 80;
    if (duration > 400) duration = 400; // keep interaction short

    // Interaction: no hispeed override, no GPU (safe)
    apply_boost_timed(
        /*sched*/ 1,
        /*cpu_boost*/ 1,
        /*cpu0_hispeed*/ -1,
        /*cpu4_hispeed*/ -1,
        /*gpu_min*/ -1,
        duration
    );
}

static int process_activity_launch_hint(void* data) {
    (void)data;

    // Launch: longer + hispeed to max + GPU min_pwrlevel to 0
    // If GPU default pwrlevel is already 0, this is no-op.
    int gpu_target = 0;
    if (g_gpu_default_pwr >= 0 && g_gpu_default_pwr < gpu_target) {
        gpu_target = g_gpu_default_pwr;
    }

    apply_boost_timed(
        /*sched*/ 1,
        /*cpu_boost*/ 1,
        /*cpu0_hispeed*/ g_cpu0_max,
        /*cpu4_hispeed*/ g_cpu4_max,
        /*gpu_min*/ gpu_target,
        /*duration*/ 2000
    );

    return HINT_HANDLED;
}

static inline int hint_is_interaction(power_hint_t hint) {
    int v = (int)hint;
    return (hint == POWER_HINT_INTERACTION) || (v == 1) || (v == 2);
}

static inline int hint_is_launch(power_hint_t hint) {
    int v = (int)hint;
    return (hint == POWER_HINT_LAUNCH) || (v == 7) || (v == 8);
}

int power_hint_override(power_hint_t hint, void* data) {
    // Keep behavior consistent with shamrock aggressive file:
    // ignore extra hints unless you explicitly add them.
    if (hint_is_interaction(hint)) {
        process_interaction_hint(data);
        return HINT_HANDLED;
    }
    if (hint_is_launch(hint)) {
        return process_activity_launch_hint(data);
    }
    return HINT_NONE;
}

int set_interactive_override(int on) {
    // When display goes off, restore immediately (avoid holding boosts during suspend)
    if (!on) restore_now();
    return HINT_HANDLED;
}

void init_platform(void) {
    // cache defaults + read max freqs
    pthread_mutex_lock(&g_lock);
    cache_defaults_locked();

    if (path_exists(kCpu0MaxFreqPath)) {
        int v;
        if (read_int(kCpu0MaxFreqPath, &v) == 0 && v > 0) g_cpu0_max = v;
    }
    if (path_exists(kCpu4MaxFreqPath)) {
        int v;
        if (read_int(kCpu4MaxFreqPath, &v) == 0 && v > 0) g_cpu4_max = v;
    }

    pthread_mutex_unlock(&g_lock);

    ALOGI("init_platform: cpu0_max=%d cpu4_max=%d sched_boost=%s kgsl=%s",
          g_cpu0_max, g_cpu4_max,
          path_exists(kSchedBoostPath) ? "yes" : "no",
          path_exists(kKgslBasePath) ? "yes" : "no");
}

// --------------------------- low-level IO ------------------------------------

static int read_int(const char* path, int* out) {
    if (!path || !out) return -EINVAL;
    int fd = TEMP_FAILURE_RETRY(open(path, O_RDONLY | O_CLOEXEC));
    if (fd < 0) return -errno;

    char buf[64];
    int n = (int)TEMP_FAILURE_RETRY(read(fd, buf, sizeof(buf) - 1));
    close(fd);
    if (n <= 0) return -EIO;
    buf[n] = '\0';
    *out = atoi(buf);
    return 0;
}

static int read_str(const char* path, char* out, size_t out_sz) {
    if (!path || !out || out_sz == 0) return -EINVAL;
    int fd = TEMP_FAILURE_RETRY(open(path, O_RDONLY | O_CLOEXEC));
    if (fd < 0) return -errno;

    int n = (int)TEMP_FAILURE_RETRY(read(fd, out, out_sz - 1));
    close(fd);
    if (n <= 0) return -EIO;
    out[n] = '\0';
    return 0;
}

static int write_str_file(const char* path, const char* s) {
    if (!path || !s) return -EINVAL;
    int fd = TEMP_FAILURE_RETRY(open(path, O_WRONLY | O_CLOEXEC));
    if (fd < 0) return -errno;

    int len = (int)strlen(s);
    int rc = (int)TEMP_FAILURE_RETRY(write(fd, s, len));
    close(fd);
    if (rc < 0) return -errno;
    return 0;
}

static int write_int(const char* path, int v) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%d", v);
    return write_str_file(path, buf);
}
