#pragma once
constexpr unsigned int MAP_Y_AXIS_MAX = 200; // y axis
constexpr unsigned int MAP_X_AXIS_MAX = 200; // x axis

constexpr unsigned int SHIP_MAX = 5;   // max ship number
constexpr unsigned int BERTH_MAX = 10; // max berth number
constexpr unsigned int ROBOT_MAX = 10; // max robot number

constexpr unsigned int FRAME_CARGO_REMAIN =
    1000; // frame length for cargo remain

constexpr unsigned int FRAME_SHIP_SWITH_FROM_BERTH =
    500; // frame length for ship switch

constexpr unsigned int FRAME_ROBOT_CRASH = 20; // frame length for robot crash

constexpr unsigned int FRAME_MAX = 15000; // max frame number

constexpr unsigned int FRAME_PER_SECOND = 50; // frame per second