/**
 * leap_publisher.c
 * ================
 * Reads finger curl data from the Leap Motion via LeapC API and publishes
 * it to stdout as a simple line protocol that the Python ROS 2 bridge reads.
 *
 * Output format (one line per frame, newline terminated):
 *   CURLS <thumb> <index> <middle> <ring> <pinky> <gesture>
 *   e.g.: CURLS 12.3 5.1 78.2 82.0 80.5 FIST
 *
 * The Python bridge reads this from a subprocess pipe and publishes to ROS 2.
 *
 * Build:
 *   gcc -o leap_publisher leap_publisher.c -lLeapC -lm -I/usr/include/ultraleap
 *   (adjust include path to match your Leap SDK install)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <LeapC.h>

#ifdef _WIN32
#include <Windows.h>
#define millisleep(ms) Sleep(ms)
#else
#include <unistd.h>
#define millisleep(ms) usleep((ms) * 1000)
#endif

#define TARGET_FPS      60
#define FRAME_TIME_MS   (1000.0f / TARGET_FPS)

// ── Gesture types ─────────────────────────────────────────────────────────────
typedef enum {
    GESTURE_UNKNOWN,
    GESTURE_OPEN_HAND,
    GESTURE_FIST,
    GESTURE_POINTING,
    GESTURE_THUMBS_UP,
    GESTURE_PEACE_SIGN,
    GESTURE_OK_SIGN
} GestureType;

typedef struct {
    GestureType type;
    float confidence;
} DetectedGesture;

// ── Math helpers ──────────────────────────────────────────────────────────────
typedef struct { float x, y, z; } Vec3;
typedef struct { float x, y, z, w; } Quat;

float vec3_distance(Vec3 a, Vec3 b) {
    float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

float angle_between_vectors(Vec3 a, Vec3 b) {
    float la = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
    float lb = sqrtf(b.x*b.x + b.y*b.y + b.z*b.z);
    if (la < 1e-4f || lb < 1e-4f) return 0.0f;
    float dot = (a.x*b.x + a.y*b.y + a.z*b.z) / (la * lb);
    dot = dot < -1.0f ? -1.0f : dot > 1.0f ? 1.0f : dot;
    return acosf(dot) * 180.0f / (float)M_PI;
}

// ── Finger curl (0 = straight, ~90 = fully curled) ────────────────────────────
float calculate_finger_curl(LEAP_HAND* hand, int finger) {
    if (finger < 0 || finger >= 5) return 0.0f;
    LEAP_DIGIT* d = &hand->digits[finger];
    Vec3 mc_px = { d->bones[1].prev_joint.x - d->bones[0].prev_joint.x,
                   d->bones[1].prev_joint.y - d->bones[0].prev_joint.y,
                   d->bones[1].prev_joint.z - d->bones[0].prev_joint.z };
    Vec3 px_im = { d->bones[2].prev_joint.x - d->bones[1].prev_joint.x,
                   d->bones[2].prev_joint.y - d->bones[1].prev_joint.y,
                   d->bones[2].prev_joint.z - d->bones[1].prev_joint.z };
    Vec3 im_ds = { d->bones[3].prev_joint.x - d->bones[2].prev_joint.x,
                   d->bones[3].prev_joint.y - d->bones[2].prev_joint.y,
                   d->bones[3].prev_joint.z - d->bones[2].prev_joint.z };
    return (angle_between_vectors(mc_px, px_im) + angle_between_vectors(px_im, im_ds)) / 2.0f;
}

float calculate_finger_spread(LEAP_HAND* hand, int f1, int f2) {
    Vec3 t1 = { hand->digits[f1].bones[3].next_joint.x,
                hand->digits[f1].bones[3].next_joint.y,
                hand->digits[f1].bones[3].next_joint.z };
    Vec3 t2 = { hand->digits[f2].bones[3].next_joint.x,
                hand->digits[f2].bones[3].next_joint.y,
                hand->digits[f2].bones[3].next_joint.z };
    return vec3_distance(t1, t2);
}

// ── Gesture detection ─────────────────────────────────────────────────────────
const char* gesture_name(GestureType g) {
    switch (g) {
        case GESTURE_OPEN_HAND:  return "OPEN_HAND";
        case GESTURE_FIST:       return "FIST";
        case GESTURE_POINTING:   return "POINTING";
        case GESTURE_THUMBS_UP:  return "THUMBS_UP";
        case GESTURE_PEACE_SIGN: return "PEACE_SIGN";
        case GESTURE_OK_SIGN:    return "OK_SIGN";
        default:                 return "UNKNOWN";
    }
}

DetectedGesture detect_gesture(LEAP_HAND* hand) {
    DetectedGesture result = {GESTURE_UNKNOWN, 0.0f};
    float curls[5];
    for (int i = 0; i < 5; i++) curls[i] = calculate_finger_curl(hand, i);

    float spread_im = calculate_finger_spread(hand, 1, 2);
    float OPEN   = 60.0f;
    float CLOSED = 50.0f;
    float SPREAD = 25.0f;

    int open_count = 0, closed_count = 0;
    for (int i = 0; i < 5; i++) {
        if (curls[i] < OPEN)   open_count++;
        if (curls[i] > CLOSED) closed_count++;
    }

    if (open_count >= 4 && spread_im > SPREAD)
        return (DetectedGesture){GESTURE_OPEN_HAND, 0.90f};
    if (closed_count >= 4)
        return (DetectedGesture){GESTURE_FIST, 0.90f};
    if (curls[1] < OPEN && curls[2] > CLOSED*0.7f && curls[3] > CLOSED*0.7f && curls[4] > CLOSED*0.7f)
        return (DetectedGesture){GESTURE_POINTING, 0.85f};
    if (curls[1] < OPEN && curls[2] < OPEN && curls[3] > CLOSED*0.6f && curls[4] > CLOSED*0.6f && spread_im > SPREAD*0.8f)
        return (DetectedGesture){GESTURE_PEACE_SIGN, 0.85f};
    if (curls[0] < OPEN+20.0f && curls[1] > CLOSED*0.6f && curls[2] > CLOSED*0.6f && curls[3] > CLOSED*0.6f && curls[4] > CLOSED*0.6f)
        return (DetectedGesture){GESTURE_THUMBS_UP, 0.80f};
    if (curls[0] > 35.0f && curls[0] < 75.0f && curls[1] > 35.0f && curls[1] < 75.0f &&
        curls[2] < OPEN && curls[3] < OPEN && curls[4] < OPEN)
        return (DetectedGesture){GESTURE_OK_SIGN, 0.80f};

    return result;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(void) {
    LEAP_CONNECTION connection;
    if (LeapCreateConnection(NULL, &connection) != eLeapRS_Success) {
        fprintf(stderr, "ERROR: Could not create Leap connection\n");
        return 1;
    }
    if (LeapOpenConnection(connection) != eLeapRS_Success) {
        fprintf(stderr, "ERROR: Could not open Leap connection\n");
        return 1;
    }

    fprintf(stderr, "Leap Motion connected — streaming finger curls\n");

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t last_frame_ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;

    while (1) {
        LEAP_CONNECTION_MESSAGE msg;
        eLeapRS rs = LeapPollConnection(connection, 1000, &msg);
        if (rs != eLeapRS_Success || msg.type != eLeapEventType_Tracking) {
            millisleep(2);
            continue;
        }

        const LEAP_TRACKING_EVENT* frame = msg.tracking_event;
        if (!frame) { millisleep(2); continue; }

        clock_gettime(CLOCK_MONOTONIC, &ts);
        int64_t now_ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
        float dt = (float)(now_ms - last_frame_ms);
        if (dt < FRAME_TIME_MS) { millisleep((uint32_t)(FRAME_TIME_MS - dt)); continue; }
        last_frame_ms = now_ms;

        if (frame->nHands > 0) {
            LEAP_HAND* hand = &frame->pHands[0];
            float curls[5];
            for (int i = 0; i < 5; i++) curls[i] = calculate_finger_curl(hand, i);
            DetectedGesture g = detect_gesture(hand);

            // Output: CURLS thumb index middle ring pinky gesture
            printf("CURLS %.2f %.2f %.2f %.2f %.2f %s\n",
                   curls[0], curls[1], curls[2], curls[3], curls[4],
                   gesture_name(g.type));
            fflush(stdout);
        } else {
            printf("NO_HAND\n");
            fflush(stdout);
        }
    }

    LeapCloseConnection(connection);
    LeapDestroyConnection(connection);
    return 0;
}
