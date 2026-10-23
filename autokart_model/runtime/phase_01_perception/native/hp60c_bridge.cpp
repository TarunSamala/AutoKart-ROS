#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>
#include "as_camera_sdk_api.h"

// A one-frame mailbox. SDK-owned pointers are copied before the callback returns.
struct Bridge {
    std::mutex mutex;
    std::condition_variable cv;
    AS_CAM_PTR camera = nullptr;
    bool connected = false, streaming = false, fresh = false;
    uint64_t drops = 0, received = 0, first_arrival_ns = 0, generation = 0;
    int width = 0, height = 0, alignment = -1;
    uint32_t rgb_id = 0, depth_id = 0;
    uint64_t rgb_ts = 0, depth_ts = 0, arrival_ns = 0;
    AS_CAM_Parameter_s calibration{};
    std::vector<uint8_t> rgb;
    std::vector<uint16_t> depth;
};
static Bridge bridge;
static std::mutex lifecycle_mutex;
static bool initialized = false;
static uint64_t now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
static void frame_cb(AS_CAM_PTR cam, const AS_SDK_Data_s *data, void *) {
    const auto &r = data->rgbImg, &d = data->depthImg;
    if (r.width == 0 || r.height == 0 || r.width != d.width || r.height != d.height ||
        r.width > 1920 || r.height > 1080 || r.size != r.width*r.height*3 ||
        d.size != d.width*d.height*2 || !r.data || !d.data) return;
    // Keep only SDK callback pairs. Their time units are vendor-defined; frame IDs and
    // timestamps are exported for diagnosis, while arrival uses monotonic nanoseconds.
    std::lock_guard<std::mutex> lock(bridge.mutex);
    if (cam != bridge.camera || !bridge.streaming) return;
    if (bridge.fresh) ++bridge.drops;
    bridge.width = r.width; bridge.height = r.height;
    bridge.rgb.assign(static_cast<const uint8_t *>(r.data), static_cast<const uint8_t *>(r.data)+r.size);
    bridge.depth.resize(d.width*d.height);
    std::memcpy(bridge.depth.data(), d.data, d.size);
    bridge.rgb_id = r.frameId; bridge.depth_id = d.frameId;
    bridge.rgb_ts = r.ts; bridge.depth_ts = d.ts;
    bridge.arrival_ns = now_ns();
    if (!bridge.received) bridge.first_arrival_ns = bridge.arrival_ns;
    ++bridge.received; ++bridge.generation; bridge.fresh = true;
    bridge.cv.notify_one();
}
static void attached(AS_CAM_ATTR_S *attr, void *) {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    AS_CAM_PTR cam = nullptr;
    if (AS_SDK_CreateCamHandle(cam, attr) != 0) return;
    AS_SDK_CAM_MODEL_E model = AS_SDK_CAM_MODEL_UNKNOWN;
    if (AS_SDK_GetCameraModel(cam, model) != 0 || model != AS_SDK_CAM_MODEL_HP60C) {
        AS_SDK_DestoryCamHandle(cam); return;
    }
    {
        std::lock_guard<std::mutex> lock(bridge.mutex);
        if (bridge.camera) { AS_SDK_DestoryCamHandle(cam); return; }
        bridge.camera = cam;
    }
    // The caller passes the exact HP60C configuration path through a global below.
}
static std::string config_path;
static void detached(AS_CAM_ATTR_S *attr, void *) {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    (void)attr;
    AS_CAM_PTR cam;
    {
        std::lock_guard<std::mutex> lock(bridge.mutex);
        cam = bridge.camera;
        bridge.streaming = false; bridge.connected = false; bridge.fresh = false;
        bridge.cv.notify_all();
    }
    if (!cam) return;
    AS_SDK_StopStream(cam); AS_SDK_CloseCamera(cam); AS_SDK_DestoryCamHandle(cam);
    std::lock_guard<std::mutex> lock(bridge.mutex);
    if (bridge.camera == cam) bridge.camera = nullptr;
}
extern "C" {
struct BridgeMeta {
    int width, height, alignment, connected;
    uint32_t rgb_id, depth_id;
    uint64_t rgb_ts, depth_ts, arrival_ns, drops, received, capture_elapsed_ns;
    float calibration[32];
};
int hp_start(const char *config) {
    if (initialized || !config) return -1;
    { std::lock_guard<std::mutex> lock(bridge.mutex);
      bridge.drops=0; bridge.received=0; bridge.first_arrival_ns=0;
      bridge.generation=0; bridge.fresh=false; bridge.alignment=-1;
      bridge.calibration=AS_CAM_Parameter_s{};
    }
    config_path = config;
    if (AS_SDK_Init() != 0) return -2;
    initialized = true;
    AS_LISTENER_CALLBACK_S cb{}; cb.onAttached = attached; cb.onDetached = detached;
    if (AS_SDK_StartListener(cb, AS_LISTENNER_TYPE_USB, true) != 0) { AS_SDK_Deinit(); initialized=false; return -3; }
    AS_CAM_PTR cam;
    { std::lock_guard<std::mutex> lock(bridge.mutex); cam = bridge.camera; }
    if (!cam) return 0; // Listener remains active for hot plug.
    return 0;
}
// Start an enumerated/hot-plugged camera outside listener callback to keep lifecycle simple.
int hp_connect() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    AS_CAM_PTR cam;
    bool streaming;
    { std::lock_guard<std::mutex> lock(bridge.mutex); cam = bridge.camera; streaming=bridge.streaming; if (!cam) return 0; }
    if (streaming) {
        // HP60C calibration may become available only after stream startup.
        AS_CAM_Parameter_s p{}; AS_CONFIG_INFO_S info{};
        int calibrated=AS_SDK_GetCamParameter(cam,&p);
        int configured=AS_SDK_GetConfigInfo(cam,info);
        std::lock_guard<std::mutex> lock(bridge.mutex);
        if (calibrated==0 && p.fxrgb>0 && p.fyrgb>0) bridge.calibration=p;
        if (configured==0) bridge.alignment=info.is_Registration ? 1 : 0;
        return bridge.calibration.fxrgb>0 && bridge.calibration.fyrgb>0 ? 1 : 2;
    }
    if (AS_SDK_OpenCamera(cam, config_path.c_str()) != 0) return -1;
    AS_SDK_SetTimeStampType(cam, AS_TIME_STAMP_TYPE_STEADY_CLOCK);
    AS_CAM_Parameter_s p{}; AS_CONFIG_INFO_S info{};
    int calibrated = AS_SDK_GetCamParameter(cam, &p);
    int configured = AS_SDK_GetConfigInfo(cam, info);
    AS_CAM_Stream_Cb_s cb{}; cb.callback = frame_cb;
    if (AS_SDK_RegisterStreamCallback(cam, &cb) != 0) { AS_SDK_CloseCamera(cam); return -2; }
    { std::lock_guard<std::mutex> lock(bridge.mutex);
      bridge.calibration = p; bridge.alignment = configured == 0 ? (info.is_Registration ? 1 : 0) : -1;
      bridge.connected = true; bridge.streaming = true;
    }
    if (AS_SDK_StartStream(cam, DEFAULT_IMG_FLG) != 0) {
        { std::lock_guard<std::mutex> lock(bridge.mutex); bridge.streaming=false; bridge.connected=false; }
        AS_SDK_CloseCamera(cam); return -3;
    }
    return calibrated == 0 ? 1 : 2; // 2: stream active but calibration unavailable
}
int hp_read(uint8_t *rgb, uint16_t *depth, unsigned capacity_pixels, BridgeMeta *meta, int timeout_ms) {
    if (!rgb || !depth || !meta) return -1;
    std::unique_lock<std::mutex> lock(bridge.mutex);
    bridge.cv.wait_for(lock, std::chrono::milliseconds(std::max(0,timeout_ms)), []{return bridge.fresh || !bridge.connected;});
    if (!bridge.fresh) return bridge.connected ? 0 : -2;
    unsigned pixels = bridge.width*bridge.height;
    if (pixels > capacity_pixels) return -3;
    std::memcpy(rgb, bridge.rgb.data(), pixels*3);
    std::memcpy(depth, bridge.depth.data(), pixels*2);
    *meta = {bridge.width, bridge.height, bridge.alignment, bridge.connected,
        bridge.rgb_id, bridge.depth_id, bridge.rgb_ts, bridge.depth_ts, bridge.arrival_ns,
        bridge.drops, bridge.received, bridge.arrival_ns-bridge.first_arrival_ns, {}};
    std::memcpy(meta->calibration, &bridge.calibration, sizeof(bridge.calibration));
    bridge.fresh = false;
    return 1;
}
void hp_stop() {
    if (!initialized) return;
    AS_SDK_StopListener();
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    AS_CAM_PTR cam;
    { std::lock_guard<std::mutex> lock(bridge.mutex); cam=bridge.camera; bridge.camera=nullptr;
      bridge.connected=false; bridge.streaming=false; bridge.fresh=false; bridge.cv.notify_all(); }
    if (cam) { AS_SDK_StopStream(cam); AS_SDK_CloseCamera(cam); AS_SDK_DestoryCamHandle(cam); }
    AS_SDK_Deinit(); initialized=false;
}
}
