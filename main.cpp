#include "def.h"
#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

// Order is IMPORTANT!
enum class Direction : uint8_t {
  right = 0,
  left = 1,
  up = 2,
  down = 3,
  none,
};

Direction reverse(Direction d) {
  switch (d) {
  case Direction::right:
    return Direction::left;
  case Direction::left:
    return Direction::right;
  case Direction::up:
    return Direction::down;
  case Direction::down:
    return Direction::up;
  default:
    return Direction::none;
  }
}

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

class Berth {
public:
  uint32_t _id;
  uint32_t _x, _y;
  uint32_t _time;     // 该泊位轮船运输到虚拟点的时间
  uint32_t _velocity; // 该泊位的装载速度
};
std::array<Berth, BERTH_MAX> berths;

class Map {
public:
  enum class Type : uint8_t {
    barrier,
    ocean,
    space,
    berth,
  };

  static char _rawmap[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
  static void readmap() {
    for (uint32_t i = 0; i < MAP_X_AXIS_MAX; ++i) {
      scanf("%s", _rawmap[i]);
      /*
‘.’ : 空地
‘*’ : 海洋
‘#’ : 障碍
‘A’ : 机器人起始位置,总共 10 个。
‘B’ : 大小为 4*4,表示泊位的位置,泊位标号在后泊位处初始化。
      */
    }
    processRawmap();
  }

  static struct Grid {
    Type _type;
    std::bitset<sizeof(uint32_t) * 8>
        _connected_berth; // if _type is berth, store self berth id, if _type
                          // is space, store connected berth ids
  } _grids[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];

  static void processRawmap() {
    for (uint32_t i = 0; i < MAP_X_AXIS_MAX; ++i) {
      for (uint32_t j = 0; j < MAP_Y_AXIS_MAX; ++j) {
        switch (_rawmap[i][j]) {
        case '.':
          _grids[i][j]._type = Type::space;
          break;
        case '*':
          _grids[i][j]._type = Type::ocean;
          break;
        case '#':
          _grids[i][j]._type = Type::barrier;
          break;
        case 'A':
          _grids[i][j]._type = Type::space;
          break;
        case 'B':
          _grids[i][j]._type = Type::berth;
          break;
        default:
          _grids[i][j]._type = Type::barrier;
          break;
        };
      }
    }
  }

  // 四个方向的坐标变化, x 0,y 1
  static const int _sdir[4][2];

  /*
    enum class Direction {
      right = 0,
      left = 1,
      up = 2,
      down = 3,
    };
  */

  static bool isMoveAble(uint32_t x, uint32_t y) {
    if (x >= MAP_X_AXIS_MAX or y >= MAP_Y_AXIS_MAX)
      return false;
    switch (_grids[x][y]._type) {
    case Type::berth:
    case Type::space:
      return true;
    default:
      return false;
    }
  }

  static Direction _toBerth[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX][BERTH_MAX];

  static void processConnectedBerth(uint32_t berth_id) {
    assert(berth_id < BERTH_MAX);

    static bool visited[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
    memset(visited, 0, sizeof(visited));
    // Berth 4*4
    for (uint32_t i = 0; i < 4; ++i) {
      for (uint32_t j = 0; j < 4; ++j) {
        uint32_t nx = berths[berth_id]._x + i;
        uint32_t ny = berths[berth_id]._y + j;
        assert(_grids[nx][ny]._type == Type::berth);
        _grids[nx][ny]._connected_berth = berth_id;
      }
    }

    // bfs
    std::queue<std::pair<uint32_t, uint32_t>> q;
    q.push({berths[berth_id]._x, berths[berth_id]._y});
    while (not q.empty()) {
      auto [nx, ny] = q.front();
      q.pop();

      if (visited[nx][ny])
        continue;
      visited[nx][ny] = true;

      switch (_grids[nx][ny]._type) {
      case Type::space:
        _grids[nx][ny]._connected_berth.set(berth_id);
        [[fallthrough]];
      case Type::berth:
        for (int i = 0; i < 4; i++) {
          uint32_t dx = nx + _sdir[i][0];
          uint32_t dy = ny + _sdir[i][1];
          if (isMoveAble(dx, dy) and not visited[dx][dy]) {
            _toBerth[dx][dy][berth_id] = reverse(static_cast<Direction>(i));
            q.push({dx, dy});
          }
        }
        break;
      case Type::ocean:
      case Type::barrier:
      default:
        break;
      }
    }
  }

} map;

char Map::_rawmap[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
Map::Grid Map::_grids[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
Direction Map::_toBerth[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX][BERTH_MAX];

// Order is IMPORTANT!
const int Map::_sdir[4][2] = {{0, 1},  // 右
                              {0, -1}, // 左
                              {-1, 0}, // 上
                              {1, 0}}; // 下

class Ship {
public:
  static uint32_t capacity;

  uint32_t _id;
  enum class Status : uint8_t {
    moving = 0,
    normal = 1,
    waiting = 2,
  } _status;

  int _target_berth_id;
  uint32_t _berth_now_id;
  uint32_t _size;

  void ship(uint32_t berth_target_id) {
    // printf("ship %u %u\n", _id, berth_id);
    assert(berth_target_id < BERTH_MAX);
    ships_actions.push_back(
        {ShipAction::ActionType::ship,
         {static_cast<uint8_t>(_id), static_cast<uint8_t>(berth_target_id)}});
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

  enum class Status {
    recovering = 0,
    normal = 1,
  } _status;

  uint32_t _carry_cargo_id;

  void move(Direction direction) {
    if (direction == Direction::none)
      return;

    uint32_t tmp = static_cast<uint32_t>(direction);
    assert(tmp < 4);

    uint32_t dx = _x + Map::_sdir[tmp][0];
    uint32_t dy = _y + Map::_sdir[tmp][1];

    assert(Map::isMoveAble(dx, dy));

    _x = dx;
    _y = dy;

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

class Cargo {
public:
  uint32_t _id;
  uint32_t _x, _y;
  uint32_t _price; // 货物的价值
  bool operator<(const Cargo &rhs) const { return _price < rhs._price; }
  bool operator==(const Cargo &rhs) const {
    return std::tie(_id, _x, _y, _price) ==
           std::tie(rhs._id, rhs._x, rhs._y, rhs._price);
  }
};

std::array<Robot, ROBOT_MAX> robots;

std::array<Ship, SHIP_MAX> ships;

// std::priority_queue<Cargo> pc;
std::vector<Cargo> cargos;

void initialization() {
  cargos.reserve(10 * FRAME_MAX);
  robots_actions.reserve(ROBOT_MAX * FRAME_MAX);
  ships_actions.reserve(SHIP_MAX * FRAME_MAX);

  std::fill_n(std::addressof(map._toBerth[0][0][0]),
              MAP_X_AXIS_MAX * MAP_Y_AXIS_MAX * BERTH_MAX, Direction::none);

  map.readmap();

  for (uint32_t i = 0; i < BERTH_MAX; i++) {
    uint32_t id, x, y, time, velocity;
    // time(1 <= time <=
    // 1000)表示该泊位轮船运输到虚拟点的时间(虚拟点移动到泊位的时间同),即产生价值的时间,时间用帧数表示。
    // Velocity(1 <= Velocity <=
    // 5)表示该泊位的装载速度,即每帧可以装载的物品数,单位是:个。
    scanf("%u %u %u %u %u", &id, &x, &y, &time, &velocity);
    berths[i] = {id, x, y, time, velocity};
    Map::processConnectedBerth(id);
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
  for (uint32_t i = 0; i < K; ++i) {
    uint32_t x, y, price;
    // 货物的位置坐标、金额
    scanf("%u %u %u", &x, &y, &price);
    cargos.push_back({static_cast<uint32_t>(cargos.size()), x, y, price});
  }

  for (uint8_t id = 0; id < ROBOT_MAX; ++id) {
    uint32_t carryflag, x, y, status;
    // carryflag: 0 表示未携带物品,1 表示携带物品。
    // 机器人所在位置坐标
    // status:  0 表示恢复状态,1 表示正常运行状态
    scanf("%u %u %u %u", &carryflag, &x, &y, &status);
    robots[id] = {id, x, y, static_cast<Robot::Status>(status), carryflag};
    if (carryflag == 0) {
      robots[id]._carry_cargo_id = -1;
    } else {
      //! TBD
    }
  }

  for (uint32_t id = 0; id < SHIP_MAX; ++id) {
    uint32_t status;
    //  0 表示移动(运输)中, 1 表示正常运行状态(即装货状态或运输完成状态),
    //  2 表示泊位外等待状态
    int berth_id; // (0 <= berth_id < 10),如果目标泊位是虚拟点,则为-1
    scanf("%u %d", &status, &berth_id);
    ships[id]._id = id;
    ships[id]._status = static_cast<Ship::Status>(status);
    ships[id]._target_berth_id = berth_id;
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

uint32_t frame_no = 1;

int main() {
  // Using C style IO , for performance

#ifdef DEBUG
  {
    auto p =
        freopen("/home/delta/workspace/huawei2024/maps/map1.txt", "r", stdin);
    assert(p);
    *p = *p;
  }
#endif

  initialization();

  for (; frame_no <= FRAME_MAX; ++frame_no) {
    auto ret = frameInput();
    assert(ret == frame_no);
    ret = ret;

    frameUpdate();

    frameOutput();
    frameDone();
  }
  return 0;
}