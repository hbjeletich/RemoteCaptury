// bridge.cpp
// Connects to Captury Live, receives poses via callback, prints to stdout.
//
// ---- stdout protocol (one line per message) ----
//   READY
//   ACTOR      <id> <numJoints> <name1>:<parent1> <name2>:<parent2> ...
//   ACTOR_NAME <id> <name>
//   POSE       <id> <timestamp_us> <numJoints> <quality> <j1> <x>..<rz> ... <foot_l> <foot_r>
//   STATUS     <id> SCALING|TRACKING|STOPPED|DELETED|UNKNOWN
//   ANGLE      <id> <angleName> <degrees>
//   CAMERA     <id> <name>
//   RECORDING_STARTED
//   RECORDING_STOPPED
//   ACTOR_CHANGED <id> <mode>   (kept for backward compat)
//
// ---- stdin commands ----
//   SNAP [x z heading]
//   SET_SHOT <name>
//   START_RECORDING
//   STOP_RECORDING
//
// ---- TCP image server (port 9001 by default) ----
//   Client connects, sends 4-byte LE int32 camera ID.
//   Server streams frames: [int32 width][int32 height][width*height*3 RGB bytes]
//   Client disconnect restores pose-only streaming.
//
// Build: cmake --build build --target bridge --config Release
//        (from the RemoteCaptury repo root)

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX          // prevent windows.h min/max macros
#  endif
#  ifndef _CRT_SECURE_NO_WARNINGS
#    define _CRT_SECURE_NO_WARNINGS
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <unordered_set>
#include <csignal>
#include <cstdint>
#include <vector>
#include <string>

#include "RemoteCaptury.h"

// ---- constants ------------------------------------------------------------

static constexpr int k_imagePort        = 9001;
static constexpr int k_streamFlags      =
    CAPTURY_STREAM_POSES |
    CAPTURY_STREAM_FOOT_CONTACT |
    CAPTURY_STREAM_ANGLES;
static constexpr int k_streamFlagsImage =
    k_streamFlags | CAPTURY_STREAM_IMAGES;

// All 45 biomechanical angle types (1..45) requested when streaming starts.
// Captury_startStreamingImagesAndAngles() requires an explicit list.
static uint16_t k_allAngles[] = {
     1,  2,  3,  4,  5,  6,  7,  8,  9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45,
};
static constexpr int k_numAllAngles = 45;

// Human-readable names for CAPTURY_LEFT_KNEE_FLEXION_EXTENSION (1) ..
// CAPTURY_TORSO_FLEXION (45). Index 0 is unused.
static const char* const k_angleNames[] = {
    "",
    "LeftKneeFlexion",          "LeftKneeVarusValgus",       "LeftKneeRotation",
    "LeftHipFlexion",           "LeftHipAbduction",           "LeftHipRotation",
    "LeftAnkleDorsiflexion",    "LeftAnklePronation",         "LeftAnkleRotation",
    "LeftShoulderFlexion",      "LeftShoulderTotalFlexion",   "LeftShoulderAbduction",  "LeftShoulderRotation",
    "LeftElbowFlexion",         "LeftForearmPronation",
    "LeftWristFlexion",         "LeftWristRadialDeviation",
    "RightKneeFlexion",         "RightKneeVarusValgus",      "RightKneeRotation",
    "RightHipFlexion",          "RightHipAbduction",          "RightHipRotation",
    "RightAnkleDorsiflexion",   "RightAnklePronation",        "RightAnkleRotation",
    "RightShoulderFlexion",     "RightShoulderTotalFlexion",  "RightShoulderAbduction", "RightShoulderRotation",
    "RightElbowFlexion",        "RightForearmPronation",
    "RightWristFlexion",        "RightWristRadialDeviation",
    "NeckFlexion",              "NeckRotation",               "NeckLateralBending",
    "CenterOfGravityX",         "CenterOfGravityY",           "CenterOfGravityZ",
    "HeadRotation",             "TorsoRotation",              "TorsoInclination",
    "HeadInclination",          "TorsoFlexion",
};
static constexpr int k_numAngleNames = static_cast<int>(sizeof(k_angleNames) / sizeof(k_angleNames[0]));

// ---- globals --------------------------------------------------------------

static std::atomic<bool>   g_running{true};
static std::mutex          g_printMutex;
static std::unordered_set<int> g_announcedActors;
static std::mutex          g_actorsMutex;

// Image streaming
#ifdef _WIN32
static SOCKET              g_imageClientSock = INVALID_SOCKET;
#endif
static std::mutex          g_imageSockMutex;
static std::atomic<int>    g_imageCameraId{-1};
static RemoteCaptury*      g_rc = nullptr;  // set in main, used in server thread

// ---- helpers --------------------------------------------------------------

static void println(const char* buf)
{
    std::lock_guard<std::mutex> lk(g_printMutex);
    fputs(buf, stdout);
    fputc('\n', stdout);
    fflush(stdout);
}

static const char* actorStatusStr(int mode)
{
    switch (mode) {
    case 0: return "SCALING";
    case 1: return "TRACKING";
    case 2: return "STOPPED";
    case 3: return "DELETED";
    default: return "UNKNOWN";
    }
}

// ---- pose callback --------------------------------------------------------

static void onNewPose(RemoteCaptury* /*rc*/,
                      CapturyActor*  actor,
                      CapturyPose*   pose,
                      int            trackingQuality,
                      void*          /*userArg*/)
{
    if (!actor || !pose || !pose->transforms) return;

    {
        std::lock_guard<std::mutex> lk(g_actorsMutex);
        if (g_announcedActors.find(actor->id) == g_announcedActors.end()) {
            // ACTOR line
            char buf[8192];
            int off = std::snprintf(buf, sizeof(buf),
                                    "ACTOR %d %d", actor->id, actor->numJoints);
            for (int i = 0; i < actor->numJoints && off < (int)sizeof(buf) - 80; ++i)
                off += std::snprintf(buf + off, sizeof(buf) - off,
                                     " %s:%d",
                                     actor->joints[i].name, actor->joints[i].parent);
            println(buf);

            // ACTOR_NAME line
            if (actor->name[0] != '\0') {
                char nameBuf[64];
                std::snprintf(nameBuf, sizeof(nameBuf), "ACTOR_NAME %d %s",
                              actor->id, actor->name);
                println(nameBuf);
            }

            g_announcedActors.insert(actor->id);
        }
    }

    static thread_local char buf[16384];
    int off = std::snprintf(buf, sizeof(buf),
                            "POSE %d %llu %d %d",
                            actor->id,
                            (unsigned long long)pose->timestamp,
                            pose->numTransforms,
                            trackingQuality);

    int n = pose->numTransforms;
    if (n > actor->numJoints) n = actor->numJoints;
    for (int i = 0; i < n && off < (int)sizeof(buf) - 200; ++i) {
        const CapturyTransform& t = pose->transforms[i];
        off += std::snprintf(buf + off, sizeof(buf) - off,
                             " %s %.4f %.4f %.4f %.4f %.4f %.4f",
                             actor->joints[i].name,
                             t.translation[0], t.translation[1], t.translation[2],
                             t.rotation[0],    t.rotation[1],    t.rotation[2]);
    }

    // Optional foot contact flags
    int foot_l = (pose->flags & CAPTURY_LEFT_FOOT_ON_GROUND)  ? 1 : 0;
    int foot_r = (pose->flags & CAPTURY_RIGHT_FOOT_ON_GROUND) ? 1 : 0;
    off += std::snprintf(buf + off, sizeof(buf) - off, " %d %d", foot_l, foot_r);

    println(buf);
}

// ---- angle callback -------------------------------------------------------

static void onNewAngles(RemoteCaptury*     /*rc*/,
                        const CapturyActor* actor,
                        int                 numAngles,
                        CapturyAngleData*   values,
                        void*               /*userArg*/)
{
    if (!actor || !values || numAngles == 0) return;
    char buf[128];
    for (int i = 0; i < numAngles; ++i) {
        int type = static_cast<int>(values[i].type);
        if (type >= 1 && type < k_numAngleNames) {
            std::snprintf(buf, sizeof(buf),
                          "ANGLE %d %s %.2f",
                          actor->id, k_angleNames[type], values[i].value);
            println(buf);
        }
    }
}

// ---- actor-changed callback -----------------------------------------------

static void onActorChanged(RemoteCaptury* /*rc*/, int actorId, int mode, void* /*userArg*/)
{
    // Legacy line (kept for backward compatibility with existing log handlers)
    char buf[128];
    std::snprintf(buf, sizeof(buf), "ACTOR_CHANGED %d %d", actorId, mode);
    println(buf);

    // New STATUS line that the Python app reads
    std::snprintf(buf, sizeof(buf), "STATUS %d %s", actorId, actorStatusStr(mode));
    println(buf);

    // If actor was removed, allow re-announcement if it comes back
    if (mode != ACTOR_TRACKING) {
        std::lock_guard<std::mutex> lk(g_actorsMutex);
        g_announcedActors.erase(actorId);
    }
}

// ---- image callback -------------------------------------------------------

#ifdef _WIN32
static void onImageReceived(RemoteCaptury* /*rc*/, const CapturyImage* img, void* /*userArg*/)
{
    if (!img || !img->data) return;

    std::lock_guard<std::mutex> lk(g_imageSockMutex);
    if (g_imageClientSock == INVALID_SOCKET) return;

    int32_t w = img->width;
    int32_t h = img->height;
    const uint8_t* src = img->data;

    // Send header: width, height (little-endian int32)
    char hdr[8];
    std::memcpy(hdr,     &w, 4);
    std::memcpy(hdr + 4, &h, 4);
    int sent = send(g_imageClientSock, hdr, 8, 0);
    if (sent != 8) {
        closesocket(g_imageClientSock);
        g_imageClientSock = INVALID_SOCKET;
        return;
    }

    // Send raw RGB data in chunks
    int total = w * h * 3;
    int offset = 0;
    while (offset < total) {
        int chunk = std::min(65536, total - offset);
        sent = send(g_imageClientSock,
                    reinterpret_cast<const char*>(src + offset), chunk, 0);
        if (sent <= 0) {
            closesocket(g_imageClientSock);
            g_imageClientSock = INVALID_SOCKET;
            return;
        }
        offset += sent;
    }
}

// ---- image server thread --------------------------------------------------

static void imageServerThread()
{
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    SOCKET listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock == INVALID_SOCKET) {
        std::fprintf(stderr, "[bridge/img] failed to create listen socket\n");
        return;
    }

    int yes = 1;
    setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&yes), sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(static_cast<u_short>(k_imagePort));

    if (bind(listenSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR ||
        listen(listenSock, 1) == SOCKET_ERROR)
    {
        std::fprintf(stderr, "[bridge/img] bind/listen failed on port %d\n", k_imagePort);
        closesocket(listenSock);
        return;
    }
    std::fprintf(stderr, "[bridge/img] listening on port %d\n", k_imagePort);

    while (g_running) {
        // Non-blocking accept loop so we can check g_running
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(listenSock, &rfds);
        timeval tv{1, 0};  // 1 second timeout
        int sel = select(0, &rfds, nullptr, nullptr, &tv);
        if (sel <= 0) continue;

        SOCKET client = accept(listenSock, nullptr, nullptr);
        if (client == INVALID_SOCKET) continue;

        std::fprintf(stderr, "[bridge/img] client connected\n");

        // Read 4-byte camera ID
        int32_t cameraId = 0;
        int recvd = recv(client, reinterpret_cast<char*>(&cameraId), 4, MSG_WAITALL);
        if (recvd != 4) {
            closesocket(client);
            continue;
        }
        std::fprintf(stderr, "[bridge/img] streaming camera %d\n", cameraId);

        // Switch to image streaming for the requested camera
        if (g_rc) {
            Captury_stopStreaming(g_rc);
            Captury_registerImageStreamingCallback(g_rc, onImageReceived, nullptr);
            int imgOk = Captury_startStreamingImagesAndAngles(g_rc, k_streamFlagsImage,
                                                              cameraId,
                                                              k_numAllAngles, k_allAngles);
            if (!imgOk) {
                char* err = Captury_getLastErrorMessage(g_rc);
                std::fprintf(stderr,
                    "[bridge/img] startStreamingImagesAndAngles(camera=%d) FAILED: %s\n",
                    cameraId, err ? err : "(no error message)");
                Captury_freeErrorMessage(err);
                // Fall back to non-image streaming so poses keep working
                Captury_startStreamingImagesAndAngles(g_rc, k_streamFlags, -1,
                                                      k_numAllAngles, k_allAngles);
                closesocket(client);
                continue;
            }
            std::fprintf(stderr, "[bridge/img] image streaming started for camera %d\n", cameraId);
            g_imageCameraId.store(cameraId);
        }

        {
            std::lock_guard<std::mutex> lk(g_imageSockMutex);
            g_imageClientSock = client;
        }

        // Wait until client disconnects (callback closes socket on error)
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::lock_guard<std::mutex> lk(g_imageSockMutex);
            if (g_imageClientSock == INVALID_SOCKET) break;
            // Ping with a zero-length send to detect broken connection
            if (send(g_imageClientSock, "", 0, 0) == SOCKET_ERROR) {
                closesocket(g_imageClientSock);
                g_imageClientSock = INVALID_SOCKET;
                break;
            }
        }

        std::fprintf(stderr, "[bridge/img] client disconnected\n");

        // Restore pose + angle streaming (no images)
        if (g_rc) {
            Captury_stopStreaming(g_rc);
            Captury_registerImageStreamingCallback(g_rc, nullptr, nullptr);
            Captury_startStreamingImagesAndAngles(g_rc, k_streamFlags, -1,
                                                  k_numAllAngles, k_allAngles);
            g_imageCameraId.store(-1);
        }
    }

    closesocket(listenSock);
    WSACleanup();
}
#endif  // _WIN32

// ---- signal handling ------------------------------------------------------

static void handleSignal(int /*sig*/) { g_running = false; }

// ---- main -----------------------------------------------------------------

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <captury_host_ip> [port=2101]\n", argv[0]);
        return 1;
    }

    const char* host = argv[1];
    unsigned short port = 2101;
    if (argc >= 3)
        port = static_cast<unsigned short>(std::atoi(argv[2]));

    setvbuf(stderr, nullptr, _IONBF, 0);
    std::signal(SIGINT,  handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::fprintf(stderr, "[bridge] connecting to %s:%u...\n", host, port);

    RemoteCaptury* rc = Captury_create();
    if (!rc) {
        std::fprintf(stderr, "[bridge] Captury_create() failed\n");
        return 2;
    }
    g_rc = rc;

    if (!Captury_connect(rc, host, port)) {
        std::fprintf(stderr, "[bridge] Captury_connect() failed\n");
        Captury_destroy(rc);
        return 3;
    }
    std::fprintf(stderr, "[bridge] connected.\n");

    Captury_registerActorChangedCallback(rc, onActorChanged, nullptr);
    Captury_registerNewPoseCallback(rc,     onNewPose,        nullptr);
    Captury_registerNewAnglesCallback(rc,   onNewAngles,      nullptr);

    // Angles require Captury_startStreamingImagesAndAngles with explicit angle list;
    // Captury_startStreaming with CAPTURY_STREAM_ANGLES flag alone does not work.
    if (!Captury_startStreamingImagesAndAngles(rc, k_streamFlags, -1,
                                               k_numAllAngles, k_allAngles)) {
        std::fprintf(stderr, "[bridge] Captury_startStreamingImagesAndAngles() failed\n");
        Captury_disconnect(rc);
        Captury_destroy(rc);
        return 4;
    }
    std::fprintf(stderr, "[bridge] streaming started (poses + foot contact + all 45 angles).\n");
    println("READY");

    // Query cameras AFTER streaming starts.  If the server returns 0 cameras on
    // the first try (it may still be building its list), retry once with a fresh
    // request to give it more time.
    {
        const CapturyCamera* cams = nullptr;
        int numCams = Captury_getCameras(rc, &cams, 2000);
        if (numCams == 0) {
            std::fprintf(stderr, "[bridge] 0 cameras at 2s, retrying...\n");
            numCams = Captury_getCameras(rc, &cams, 3000);
        }
        std::fprintf(stderr, "[bridge] %d camera(s) found\n", numCams);
        for (int i = 0; i < numCams; ++i) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "CAMERA %d %s",
                          cams[i].id, cams[i].name);
            println(buf);
        }
    }

#ifdef _WIN32
    std::thread imgThread(imageServerThread);
    imgThread.detach();
#endif

    // Stdin command thread
    std::thread stdinThread([rc]() {
        char line[256];
        while (g_running && std::fgets(line, sizeof(line), stdin) != nullptr) {
            size_t len = std::strlen(line);
            if (len > 0 && line[len-1] == '\n') line[--len] = '\0';
            if (len > 0 && line[len-1] == '\r') line[--len] = '\0';
            if (len == 0) continue;

            if (std::strncmp(line, "SNAP", 4) == 0) {
                float x = 0.f, z = 0.f, heading = 500.f;
                std::sscanf(line + 4, " %f %f %f", &x, &z, &heading);
                int ok = Captury_snapActor(rc, x, z, heading);
                std::fprintf(stderr, "[bridge] SNAP %.1f %.1f %.1f -> %s\n",
                             x, z, heading, ok ? "sent" : "failed");

            } else if (std::strncmp(line, "SET_SHOT ", 9) == 0) {
                const char* name = line + 9;
                Captury_setShotName(rc, name);
                std::fprintf(stderr, "[bridge] SET_SHOT '%s'\n", name);

            } else if (std::strcmp(line, "START_RECORDING") == 0) {
                int64_t ts = Captury_startRecording(rc);
                if (ts) {
                    println("RECORDING_STARTED");
                    std::fprintf(stderr, "[bridge] recording started at %lld\n", (long long)ts);
                } else {
                    std::fprintf(stderr, "[bridge] START_RECORDING failed\n");
                }

            } else if (std::strcmp(line, "STOP_RECORDING") == 0) {
                if (Captury_stopRecording(rc)) {
                    println("RECORDING_STOPPED");
                    std::fprintf(stderr, "[bridge] recording stopped\n");
                } else {
                    std::fprintf(stderr, "[bridge] STOP_RECORDING failed\n");
                }

            } else {
                std::fprintf(stderr, "[bridge] unknown command: %s\n", line);
            }
        }
        g_running = false;
    });
    stdinThread.detach();

    while (g_running)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::fprintf(stderr, "[bridge] shutting down...\n");
    Captury_stopStreaming(rc);
    Captury_disconnect(rc);
    Captury_destroy(rc);
    return 0;
}
