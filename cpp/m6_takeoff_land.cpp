// M6 — C++ port of M2: preflight checks → arm → takeoff 10 m → hold 10 s → land.
//
// WHAT IT DOES
//   Same flight sequence as scripts/m2_takeoff_land.py, but with MAVSDK C++:
//     1. connect to SITL (udpin://0.0.0.0:14540) and wait for an autopilot
//     2. wait until the EKF reports valid global+home position (health gate)
//     3. arm — a refusal (Action::Result != Success) is printed with its
//        reason string instead of crashing
//     4. takeoff to 10 m, poll relative altitude until reached
//     5. hold 10 s, land, wait for auto-disarm
//
// MAVLINK MESSAGES INVOLVED (identical to the Python version — the point of
// the port is the language, not the protocol):
//   HEARTBEAT, COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM, NAV_TAKEOFF,
//   NAV_LAND), COMMAND_ACK, GLOBAL_POSITION_INT, EXTENDED_SYS_STATE.
//
// C++ vs PYTHON API NOTES
//   - the Python API is async-first (await + async generators); the C++ API
//     offers both blocking calls and callback subscriptions. This port uses
//     blocking calls + polling for readability.
//   - MAVSDK C++ v3: Mavsdk requires an explicit Configuration (component
//     type GroundStation); first_autopilot() replaces the old
//     discover-system dance.
//
// BUILD & RUN
//   ./cpp/get_mavsdk.sh                 # once: fetch libmavsdk locally
//   cmake -S cpp -B cpp/build -G Ninja
//   cmake --build cpp/build
//   ./cpp/build/m6_takeoff_land

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

static constexpr float kTakeoffAltM = 10.0f;
static constexpr int kHoldS = 10;

int main()
{
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    std::cout << "Connecting to udpin://0.0.0.0:14540 ...\n";
    if (mavsdk.add_any_connection("udpin://0.0.0.0:14540") != ConnectionResult::Success) {
        std::cerr << "✗ connection setup failed\n";
        return 1;
    }

    auto system = mavsdk.first_autopilot(30.0);
    if (!system) {
        std::cerr << "✗ no autopilot found (is SITL running?)\n";
        return 1;
    }
    std::cout << "✓ Connected to PX4\n";

    Telemetry telemetry{system.value()};
    Action action{system.value()};

    std::cout << "Waiting for vehicle health ...\n";
    for (int i = 0; i < 120 && !telemetry.health_all_ok(); ++i) {
        sleep_for(seconds(1));
    }
    if (!telemetry.health_all_ok()) {
        std::cerr << "✗ vehicle never became healthy (EKF/GPS?)\n";
        return 1;
    }
    std::cout << "✓ Health OK (EKF global & home position)\n";

    action.set_takeoff_altitude(kTakeoffAltM);

    std::cout << "Arming ...\n";
    if (auto res = action.arm(); res != Action::Result::Success) {
        std::cerr << "✗ Arm refused: " << res
                  << " — check STATUSTEXT in QGC/pxh for the failing preflight check\n";
        return 1;
    }
    std::cout << "✓ Armed\n";

    std::cout << "Taking off to " << kTakeoffAltM << " m ...\n";
    if (auto res = action.takeoff(); res != Action::Result::Success) {
        std::cerr << "✗ Takeoff refused: " << res << '\n';
        return 1;
    }
    while (telemetry.position().relative_altitude_m < kTakeoffAltM - 0.5f) {
        sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "✓ Reached " << telemetry.position().relative_altitude_m << " m\n";

    std::cout << "Holding " << kHoldS << " s ...\n";
    sleep_for(seconds(kHoldS));

    std::cout << "Landing ...\n";
    if (auto res = action.land(); res != Action::Result::Success) {
        std::cerr << "✗ Land refused: " << res << '\n';
        return 1;
    }
    while (telemetry.armed()) {
        sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "✓ Touchdown — vehicle disarmed\n";
    return 0;
}
