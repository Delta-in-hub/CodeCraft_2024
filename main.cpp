#include "def.h"
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <queue>
#include <vector>

#define DEBUG

class RobotAction {
public:
  enum class ActionType : uint8_t {
    move,
    get,
    pull,
  } _action;
  uint8_t _param[2]; // move: id,direction, get/pull: id
};

class ShipAction {
public:
  enum class ActionType : uint8_t {
    ship,
    go,
  } _action;
  uint8_t _param[2]; // ship: id,berth_id, go: id
};

std::vector<RobotAction> robots_actions;
std::vector<ShipAction> ships_actions;

class Map {
public:
  enum class Type {
    space,
    ocean,
    barrier,
    robot,
    berth,
    cargo,
  };

  static char _rawmap[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
  void readmap() {
    for (int i = 0; i < MAP_X_AXIS_MAX; ++i) {
      scanf("%s", _rawmap[i]);
      /*
‘.’ : 空地
‘*’ : 海洋
‘#’ : 障碍
‘A’ : 机器人起始位置,总共 10 个。
‘B’ : 大小为 4*4,表示泊位的位置,泊位标号在后泊位处初始化。
      */
    }
  }
};

char Map::_rawmap[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];

class Ship {
public:
  static uint32_t capacity;
  uint32_t _id;

  enum class Status {
    moving = 0,
    normal = 1,
    waiting = 2,
  } _status;

  uint32_t _target_berth_id;

  void ship(uint32_t berth_id) {
    // printf("ship %u %u\n", _id, berth_id);
    ships_actions.push_back(
        {ShipAction::ActionType::ship,
         {static_cast<uint8_t>(_id), static_cast<uint8_t>(berth_id)}});
  }
  void go() {
    // printf("go %u\n", _id);
    ships_actions.push_back(
        {ShipAction::ActionType::go, {static_cast<uint8_t>(_id), 0}});
  }
};

uint32_t Ship::capacity = -1;

class Robot {
public:
  uint32_t _id;
  uint32_t _x, _y;

  enum class Direction {
    right = 0,
    left = 1,
    up = 2,
    down = 3,
  };

  enum class Status {
    recovering = 0,
    normal = 1,
  } _status;

  uint32_t _carry_cargo_id;

  void move(Direction direction) {
    switch (direction) {
    case Direction::right:
      _y++;
      break;
    case Direction::left:
      _y--;
      break;
    case Direction::up:
      _x--;
      break;
    case Direction::down:
      _x++;
      break;
    default:
      assert("Invalid direction" == nullptr);
      break;
    }

    // printf("move %u %u\n", _id, static_cast<uint32_t>(direction));
    robots_actions.push_back(
        {RobotAction::ActionType::move,
         {static_cast<uint8_t>(_id), static_cast<uint8_t>(direction)}});
  }

  void get() {
    // printf("get %u\n", _id);
    robots_actions.push_back(
        {RobotAction::ActionType::get, {static_cast<uint8_t>(_id), 0}});
  }
  void pull() {
    // printf("pull %u\n", _id);
    robots_actions.push_back(
        {RobotAction::ActionType::pull, {static_cast<uint8_t>(_id), 0}});
  }
};

class Berth {
public:
  uint32_t _id;
  uint32_t _x, _y;
  uint32_t _time;     // 该泊位轮船运输到虚拟点的时间
  uint32_t _velocity; // 该泊位的装载速度
};

class Cargo {
public:
  uint32_t _id;
  uint32_t _x, _y;
  uint32_t _price; // 货物的价值
};

Map map;

std::array<Ship, SHIP_MAX> ships;
std::array<Robot, ROBOT_MAX> robots;
std::array<Berth, BERTH_MAX> berths;

std::vector<Cargo> cargos;

void initialization() {
  cargos.reserve(10 * FRAME_MAX);
  robots_actions.reserve(ROBOT_MAX * FRAME_MAX);
  ships_actions.reserve(SHIP_MAX * FRAME_MAX);

  map.readmap();
  for (int i = 0; i < BERTH_MAX; i++) {
    uint32_t id, x, y, time, velocity;
    // time(1 <= time <=
    // 1000)表示该泊位轮船运输到虚拟点的时间(虚拟点移动到泊位的时间同),即产生价值的时间,时间用帧数表示。
    // Velocity(1 <= Velocity <=
    // 5)表示该泊位的装载速度,即每帧可以装载的物品数,单位是:个。
    scanf("%u %u %u %u %u", &id, &x, &y, &time, &velocity);
    berths[i] = {id, x, y, time, velocity};
  }

  // 船的容积,即最多能装的物品数。
  scanf("%u", &Ship::capacity);

  char okk[32];
  scanf("%s", okk);

  printf("OK\n");
  fflush(stdout);
}

uint32_t frameInput() {

  uint32_t frame_id;
  uint64_t money;
  // 表示帧序号(从 1 开始递增) 、当前金钱数
  scanf("%u %lu", &frame_id, &money);

  uint32_t K;
  // 场上新增货物的数量 K
  scanf("%u", &K);
  for (int i = 0; i < K; ++i) {
    uint32_t x, y, price;
    // 货物的位置坐标、金额
    scanf("%u %u %u", &x, &y, &price);
  }

  for (int i = 0; i < ROBOT_MAX; ++i) {
    uint32_t carryflag, x, y, status;
    // carryflag: 0 表示未携带物品,1 表示携带物品。
    // 机器人所在位置坐标
    // status:  0 表示恢复状态,1 表示正常运行状态
    scanf("%u %u %u %u", &carryflag, &x, &y, &status);
  }

  for (int id = 0; id < SHIP_MAX; ++id) {
    uint32_t status;
    //  0 表示移动(运输)中, 1 表示正常运行状态(即装货状态或运输完成状态),
    //  2 表示泊位外等待状态
    int berth_id; // (0 <= berth_id < 10),如果目标泊位是虚拟点,则为-1
    scanf("%u %d", &status, &berth_id);
  }

  char okk[32];
  scanf("%s", okk);
  return frame_id;
};

void frameUpdate() {}

void frameOutput() {
  for (auto &&action : robots_actions) {
    switch (action._action) {
    case RobotAction::ActionType::move:
      printf("move %u %u\n", action._param[0], action._param[1]);
      break;
    case RobotAction::ActionType::get:
      printf("get %u\n", action._param[0]);
      break;

    case RobotAction::ActionType::pull:
      printf("pull %u\n", action._param[0]);
      break;
    default:
      assert("Invalid action" == nullptr);
      break;
    }
  }

  for (auto &&action : ships_actions) {
    switch (action._action) {
    case ShipAction::ActionType::ship:
      printf("ship %u %u\n", action._param[0], action._param[1]);
      break;
    case ShipAction::ActionType::go:
      printf("go %u\n", action._param[0]);
      break;
    default:
      assert("Invalid action" == nullptr);
      break;
    }
  }
  printf("OK\n");
  fflush(stdout);
}

void frameDone() {
  robots_actions.clear();
  ships_actions.clear();
}

int main() {
  // Using C style IO , for performance

#ifdef DEBUG
  {
    auto p = freopen("~/workspace/huawei2024/maps/map1.txt", "r", stdin);
    assert(p);
  }
#endif

  initialization();

  for (int frame_no = 1; frame_no <= FRAME_MAX; ++frame_no) {
    auto ret = frameInput();
    assert(ret == frame_no);

    frameUpdate();

    frameOutput();
    frameDone();
  }
  return 0;
}