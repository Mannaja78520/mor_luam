#include "ros/MicroRosBridge.h"

#include <ESPmDNS.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#include <config.h>
#include "app/Settings.h"
#include "app_config.h"
#include "control/ControlLoop.h"
#include "nav/WaypointRunner.h"
#include "net/NetworkManager.h"

// ---- everything micro-ROS, in one place ----------------------------------------
namespace {

struct Ros {
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_init_options_t initOptions;
    rcl_node_t node;
    rclc_executor_t executor;

    rcl_subscription_t subMove, subSpinPid, subSteerPid;
    geometry_msgs__msg__Twist moveMsg;
    std_msgs__msg__Float32MultiArray spinPidMsg, steerPidMsg;
    float spinPidBuf[9], steerPidBuf[9];

    rcl_publisher_t pubDebugCmd, pubTicks, pubSteer, pubYaw, pubImuOk, pubState, pubOdom;
    std_msgs__msg__Int32 ticks;
    std_msgs__msg__Float32 steer, yaw;
    std_msgs__msg__Bool imuOk;
    std_msgs__msg__String state;
    geometry_msgs__msg__Twist debugCmd;
    nav_msgs__msg__Odometry odom;
    char stateBuf[160];
};

Ros ros;
micro_ros_agent_locator locator;       // read by the transport on every packet
ControlLoop* gCtrl = nullptr;
WaypointRunner* gRunner = nullptr;

bool entitiesUp = false;
IPAddress agentIp;
String agentFrom = "";                  // which candidate the address came from
uint8_t candIdx = 0;
uint32_t lastPingMs = 0, lastPubMs = 0, connectedAtMs = 0;
uint32_t connects = 0;
uint8_t pingMisses = 0;

#define RCOK(fn) (RCL_RET_OK == (fn))
// result deliberately ignored (as RCSOFTCHECK did before the refactor)
#define RCSOFT(fn) do { rcl_ret_t rc_ = (fn); (void)rc_; } while (0)
// publish() returns "not confirmed yet" with session timeout 0 - the message is still sent
#define PUBLISH(p, m) RCSOFT(rcl_publish(&(p), (m), NULL))

// ---- callbacks (run inside rclc_executor_spin_some, in loop()) ----

void onMove(const void* msgin) {
    const auto* m = static_cast<const geometry_msgs__msg__Twist*>(msgin);
    DriveCommand c;
    c.rpm = m->linear.x;
    c.headingDeg = m->angular.z;
    c.distM = m->linear.y;
    c.tolM = m->linear.z;
    gRunner->cancelForRos();            // ROS takes over from a web route
    gCtrl->command(c, CommandSource::Ros);
}

void onPid(const std_msgs__msg__Float32MultiArray* msg, bool steerLoop) {
    if (msg) gCtrl->setPid(steerLoop, msg->data.data, msg->data.size);
}
void onSpinPid(const void* m) { onPid(static_cast<const std_msgs__msg__Float32MultiArray*>(m), false); }
void onSteerPid(const void* m) { onPid(static_cast<const std_msgs__msg__Float32MultiArray*>(m), true); }

// ---- entities ----

void prepareMessages() {
    memset(&ros.spinPidMsg, 0, sizeof(ros.spinPidMsg));
    ros.spinPidMsg.data.data = ros.spinPidBuf;
    ros.spinPidMsg.data.capacity = 9;
    memset(&ros.steerPidMsg, 0, sizeof(ros.steerPidMsg));
    ros.steerPidMsg.data.data = ros.steerPidBuf;
    ros.steerPidMsg.data.capacity = 9;

    memset(&ros.odom, 0, sizeof(ros.odom));
    static char odomFrame[] = "odom";
    static char baseFrame[] = "base_link";
    ros.odom.header.frame_id.data = odomFrame;
    ros.odom.header.frame_id.size = strlen(odomFrame);
    ros.odom.header.frame_id.capacity = sizeof(odomFrame);
    ros.odom.child_frame_id.data = baseFrame;
    ros.odom.child_frame_id.size = strlen(baseFrame);
    ros.odom.child_frame_id.capacity = sizeof(baseFrame);
}

bool createEntities() {
    ros.allocator = rcl_get_default_allocator();
    ros.initOptions = rcl_get_zero_initialized_init_options();
    if (!RCOK(rcl_init_options_init(&ros.initOptions, ros.allocator))) return false;
    RCSOFT(rcl_init_options_set_domain_id(&ros.initOptions, ROS_DOMAIN_ID));
    if (!RCOK(rclc_support_init_with_options(&ros.support, 0, NULL, &ros.initOptions, &ros.allocator))) return false;
    if (!RCOK(rclc_node_init_default(&ros.node, "mor_luam_firmware", "", &ros.support))) return false;

    using namespace std;
    bool ok = true;
    // Reliable QoS (the PC nodes subscribe reliable), but publish() must not
    // wait for the agent's ACK of each message: 7 topics x one Wi-Fi round
    // trip made odom ~2 Hz. Timeout 0 = send and go; lost packets are still
    // re-sent by the reliable stream while the executor spins.
#define PUB(p, pkg, T, topic)                                                                                     \
    ok = ok && RCOK(rclc_publisher_init_default(&p, &ros.node, ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, msg, T), topic)); \
    if (ok) RCSOFT(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&p), 0))
#define SUB(s, pkg, T, topic) ok = ok && RCOK(rclc_subscription_init_default(&s, &ros.node, ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, msg, T), topic))
    PUB(ros.pubDebugCmd, geometry_msgs, Twist, "/mor_luam/debug/wheel/cmd_vel");
    PUB(ros.pubTicks, std_msgs, Int32, "/mor_luam/feedback/drive_ticks");
    PUB(ros.pubSteer, std_msgs, Float32, "/mor_luam/feedback/steer_deg");
    PUB(ros.pubYaw, std_msgs, Float32, "/mor_luam/feedback/imu_yaw_deg");
    PUB(ros.pubImuOk, std_msgs, Bool, "/mor_luam/feedback/imu_ok");
    PUB(ros.pubState, std_msgs, String, "/mor_luam/debug/esp_state");
    PUB(ros.pubOdom, nav_msgs, Odometry, "/mor_luam/odom/esp");
    SUB(ros.subMove, geometry_msgs, Twist, "/mor_luam/cmd_vel/move");
    SUB(ros.subSpinPid, std_msgs, Float32MultiArray, "/mor_luam/config/spin_pid");
    SUB(ros.subSteerPid, std_msgs, Float32MultiArray, "/mor_luam/config/steer_pid");
#undef PUB
#undef SUB
    if (!ok) return false;

    ros.executor = rclc_executor_get_zero_initialized_executor();
    ok = RCOK(rclc_executor_init(&ros.executor, &ros.support.context, 3, &ros.allocator)) &&
         RCOK(rclc_executor_add_subscription(&ros.executor, &ros.subMove, &ros.moveMsg, &onMove, ON_NEW_DATA)) &&
         RCOK(rclc_executor_add_subscription(&ros.executor, &ros.subSpinPid, &ros.spinPidMsg, &onSpinPid, ON_NEW_DATA)) &&
         RCOK(rclc_executor_add_subscription(&ros.executor, &ros.subSteerPid, &ros.steerPidMsg, &onSteerPid, ON_NEW_DATA));
    return ok;
}

void destroyEntities() {
    rmw_context_t* ctx = rcl_context_get_rmw_context(&ros.support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
    RCSOFT(rcl_subscription_fini(&ros.subMove, &ros.node));
    RCSOFT(rcl_subscription_fini(&ros.subSpinPid, &ros.node));
    RCSOFT(rcl_subscription_fini(&ros.subSteerPid, &ros.node));
    RCSOFT(rcl_publisher_fini(&ros.pubDebugCmd, &ros.node));
    RCSOFT(rcl_publisher_fini(&ros.pubTicks, &ros.node));
    RCSOFT(rcl_publisher_fini(&ros.pubSteer, &ros.node));
    RCSOFT(rcl_publisher_fini(&ros.pubYaw, &ros.node));
    RCSOFT(rcl_publisher_fini(&ros.pubImuOk, &ros.node));
    RCSOFT(rcl_publisher_fini(&ros.pubState, &ros.node));
    RCSOFT(rcl_publisher_fini(&ros.pubOdom, &ros.node));
    RCSOFT(rcl_node_fini(&ros.node));
    RCSOFT(rclc_executor_fini(&ros.executor));
    RCSOFT(rclc_support_fini(&ros.support));
    // micro-ROS keeps init options in a pool of 3 (RMW_UXRCE_MAX_OPTIONS) and
    // every connect takes one more; without this the robot could reconnect
    // twice, then never again until a reboot.
    RCSOFT(rcl_init_options_fini(&ros.initOptions));
    ros.initOptions = rcl_get_zero_initialized_init_options();
}

// Over the PC hotspot the reliable link carries ~45 messages/s, far less than
// 7 topics x 50 Hz. So each call sends ONE of the six small topics (in turn,
// first, so none of them starves) and then the odometry with what is left.
// Measured on the robot: odom ~15-25 Hz, the others a few Hz each.
void publish(const RobotState& s) {
    static uint8_t turn = 0;
    switch (turn++ % 6) {
        case 0:
            ros.ticks.data = s.ticks;
            PUBLISH(ros.pubTicks, &ros.ticks);
            break;
        case 1:
            ros.steer.data = s.steerDeg;
            PUBLISH(ros.pubSteer, &ros.steer);
            break;
        case 2:
            ros.yaw.data = s.imuYawDeg;
            PUBLISH(ros.pubYaw, &ros.yaw);
            break;
        case 3:
            ros.imuOk.data = s.imuOk;
            PUBLISH(ros.pubImuOk, &ros.imuOk);
            break;
        case 4:
            ros.debugCmd.linear.x = s.targetRpm;          // commanded rpm
            ros.debugCmd.linear.y = s.rpm;                // measured rpm
            ros.debugCmd.linear.z = s.headingDeg;         // body heading
            ros.debugCmd.angular.x = s.pwm;
            ros.debugCmd.angular.y = s.steerDeg;
            ros.debugCmd.angular.z = s.steerTargetDeg;
            PUBLISH(ros.pubDebugCmd, &ros.debugCmd);
            break;
        default:
            snprintf(ros.stateBuf, sizeof(ros.stateBuf),
                     "mode=%s, src=%s, tgt=%.1f, heading=%.1f, e_steer=%.1f, steer=%.1f->%.1f, imu_ok=%d, pwm=%d",
                     s.halted ? "HALT" : (s.driving ? "DRIVE" : "STEER"), sourceName(s.source), s.targetHeadingDeg,
                     s.headingDeg, s.steerErrDeg, s.steerDeg, s.steerTargetDeg, s.imuOk ? 1 : 0, s.pwm);
            ros.state.data.data = ros.stateBuf;
            ros.state.data.size = strlen(ros.stateBuf);
            ros.state.data.capacity = sizeof(ros.stateBuf);
            PUBLISH(ros.pubState, &ros.state);
            break;
    }

    nav_msgs__msg__Odometry& o = ros.odom;
    o.header.stamp.sec = (int32_t)(s.stampMs / 1000UL);
    o.header.stamp.nanosec = (uint32_t)((s.stampMs % 1000UL) * 1000000UL);
    o.pose.pose.position.x = s.x;
    o.pose.pose.position.y = s.y;
    o.pose.pose.position.z = 0.0;
    const double half = s.thetaRad * 0.5;
    o.pose.pose.orientation.x = 0.0;
    o.pose.pose.orientation.y = 0.0;
    o.pose.pose.orientation.z = sin(half);
    o.pose.pose.orientation.w = cos(half);
    o.twist.twist.linear.x = s.vx;
    o.twist.twist.angular.z = s.wz;
    PUBLISH(ros.pubOdom, &o);
}

}  // namespace

// ---- where is the agent? -------------------------------------------------------
// Tried in turn, one per attempt:
//   1. the address set on the web page: an IP, or a name ("my-pc" / "my-pc.local", via mDNS)
//   2. the Wi-Fi gateway: on a PC hotspot (Windows "manny") the PC is the gateway
//   3. AGENT_IP from conf_network.h
static bool nextCandidate(Settings* settings, NetworkManager* net) {
    const SettingsData s = settings->get();
    for (int tries = 0; tries < 3; ++tries) {
        const uint8_t which = candIdx++ % 3;
        IPAddress ip;
        if (which == 0) {
            if (s.agentHost.isEmpty()) continue;
            if (!ip.fromString(s.agentHost)) {
                String name = s.agentHost;
                if (name.endsWith(".local")) name = name.substring(0, name.length() - 6);
                ip = MDNS.queryHost(name.c_str(), 1500);
                if ((uint32_t)ip == 0) continue;
            }
            agentFrom = "ตั้งค่า (" + s.agentHost + ")";
        } else if (which == 1) {
            ip = net->gateway();
            if ((uint32_t)ip == 0) continue;
            agentFrom = "gateway ของ WiFi";
        } else {
            ip = AGENT_IP;
            agentFrom = "ค่าเริ่มต้น";
        }
        agentIp = ip;
        locator.address = ip;
        locator.port = s.agentPort;
        return true;
    }
    return false;
}

static Settings* gSettings = nullptr;
static NetworkManager* gNet = nullptr;

void MicroRosBridge::begin(ControlLoop* ctrl, WaypointRunner* runner, Settings* settings, NetworkManager* net) {
    gCtrl = ctrl;
    gRunner = runner;
    gSettings = settings;
    gNet = net;
    prepareMessages();
    locator.address = AGENT_IP;
    locator.port = settings->get().agentPort;
    // Our own Wi-Fi is already managed; only the UDP transport is micro-ROS's.
    rmw_uros_set_custom_transport(false, (void*)&locator, platformio_transport_open, platformio_transport_close,
                                  platformio_transport_write, platformio_transport_read);
}

void MicroRosBridge::loop() {
    const uint32_t now = millis();
    if (!gNet->connected()) {
        if (entitiesUp) {
            destroyEntities();
            entitiesUp = false;
            if (gCtrl->source() == CommandSource::Ros) gCtrl->halt("Wi-Fi หลุด (คำสั่งจาก ROS)");
        }
        state_ = State::NoWifi;
        return;
    }

    switch (state_) {
        case State::NoWifi:
            state_ = State::Waiting;
            lastPingMs = 0;
            break;

        case State::Waiting:
            if (now - lastPingMs < ROS_PING_WAITING_MS) break;
            lastPingMs = now;
            if (!nextCandidate(gSettings, gNet)) break;
            // short pings: loop() also runs the network and the route
            if (RMW_RET_OK == rmw_uros_ping_agent(200, 2)) {
                if (createEntities()) {
                    entitiesUp = true;
                    state_ = State::Connected;
                    connectedAtMs = now;
                    ++connects;
                    pingMisses = 0;
                    Serial.printf("[ros] agent %s:%u (%s)\n", agentIp.toString().c_str(), locator.port, agentFrom.c_str());
                } else {
                    destroyEntities();
                }
            }
            break;

        case State::Connected: {
            if (now - lastPingMs >= ROS_PING_CONNECTED_MS) {
                lastPingMs = now;
                // one short ping per check; lost only after several misses in a
                // row, so a busy link does not tear the session down
                if (RMW_RET_OK == rmw_uros_ping_agent(150, 1)) pingMisses = 0;
                else ++pingMisses;
                if (pingMisses >= ROS_PING_MISSES_LOST) {
                    pingMisses = 0;
                    Serial.println("[ros] agent lost");
                    destroyEntities();
                    entitiesUp = false;
                    state_ = State::Waiting;
                    candIdx = candIdx > 0 ? candIdx - 1 : 0;   // try the same address first
                    if (gCtrl->source() == CommandSource::Ros) gCtrl->halt("agent หลุด (คำสั่งจาก ROS)");
                    break;
                }
            }
            RCSOFT(rclc_executor_spin_some(&ros.executor, RCL_MS_TO_NS(2)));
            if (now - lastPubMs >= ROS_PUBLISH_PERIOD_MS) {
                lastPubMs = now;
                publish(gCtrl->snapshot());
            }
            break;
        }
    }
}

void MicroRosBridge::statusJson(JsonObject o) {
    const char* st = state_ == State::Connected ? "connected" : (state_ == State::Waiting ? "waiting" : "no-wifi");
    o["state"] = st;
    o["agent"] = agentIp.toString() + ":" + String(locator.port);
    o["from"] = agentFrom;
    o["domain"] = (int)ROS_DOMAIN_ID;
    o["connects"] = connects;
    o["upMs"] = state_ == State::Connected ? millis() - connectedAtMs : 0;
}
