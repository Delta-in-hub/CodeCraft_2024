#include "def.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

uint32_t frame_current = 1;

const uint32_t &getCurrentFrame() { return frame_current; }

// Order is IMPORTANT!
enum class Direction : uint8_t {
  right = 0,
  left = 1,
  up = 2,
  down = 3,
  none,
};

const std::array<uint8_t, 5> &getRandomDir() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::array<uint8_t, 5> dir{0, 1, 2, 3, 4};

  std::shuffle(dir.begin(), dir.end(), gen);
  return cref(dir);
}
// [l,r]
int getRandom(int l, int r) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(l, r);
  return dis(gen);
}

template <class T> std::vector<T> &randShuffle(std::vector<T> &vec) {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::shuffle(vec.begin(), vec.end(), gen);
  return ref(vec);
}

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

class TimeCounter {
public:
  TimeCounter(const char *p) {
    start = std::chrono::high_resolution_clock::now();
    funcname = p;
  }

  ~TimeCounter() {
#ifndef NDEBUG
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    // if (duration > std::chrono::milliseconds(5))
    std::cerr << funcname << " Elapsed time: " << duration.count() << " ms\n";
#endif
  }

private:
  const char *funcname = nullptr;
  std::chrono::time_point<std::chrono::high_resolution_clock> start;
};

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

// 因为先输出机器人操作,再输出船只操作,故先存起来
std::vector<RobotAction> robots_actions;
std::vector<ShipAction> ships_actions;

class Berth {
public:
  uint32_t _id;
  uint32_t _x, _y;
  uint32_t _time;     // 该泊位轮船运输到虚拟点的时间
  uint32_t _velocity; // 该泊位的装载速度
  uint32_t _cargo_ongoing;
  std::deque<std::pair<uint32_t, uint32_t>>
      _shipIds; // 该泊位的船只编号; {arrive_frame,ship id}
  std::deque<uint32_t> _cargoIds; // 该泊位的货物编号;
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
    union {
      uint32_t _berth_id; // if type is berth, then this is the id of the berth
      uint16_t _dis[BERTH_MAX]; // else this is the distance to the ith berth ,
                                // -1 表示不可达. 地图200*200,最长的路径 uint16
                                // 也足够
    };
  } _grids[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];

  static void processRawmap() {
    for (uint32_t i = 0; i < MAP_X_AXIS_MAX; ++i) {
      for (uint32_t j = 0; j < MAP_Y_AXIS_MAX; ++j) {
        std::fill_n(std::addressof(_grids[i][j]._dis[0]), BERTH_MAX, -1);
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

  static void processConnectedBerth(uint32_t berth_id) {
    assert(berth_id < BERTH_MAX);

    static bool inQueue[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
    memset(inQueue, 0, sizeof(inQueue));

    // Berth 4*4
    for (uint32_t i = 0; i < 4; ++i) {
      for (uint32_t j = 0; j < 4; ++j) {
        uint32_t nx = berths[berth_id]._x + i;
        uint32_t ny = berths[berth_id]._y + j;
        assert(_grids[nx][ny]._type == Type::berth);
        _grids[nx][ny]._berth_id = berth_id;
      }
    }

    // <x,y,step>
    using Tup = std::tuple<uint32_t, uint32_t, uint32_t>;

    // bfs
    std::queue<Tup> q;
    q.push({berths[berth_id]._x, berths[berth_id]._y, 0});
    inQueue[berths[berth_id]._x][berths[berth_id]._y] = true;

    while (not q.empty()) {
      auto [nx, ny, step] = q.front();
      q.pop();

      assert(inQueue[nx][ny]);

      switch (_grids[nx][ny]._type) {
      case Type::space:
        _grids[nx][ny]._dis[berth_id] = step;
        [[fallthrough]];
      case Type::berth:
        for (int i = 0; i < 4; i++) {
          uint32_t dx = nx + _sdir[i][0];
          uint32_t dy = ny + _sdir[i][1];
          if (isMoveAble(dx, dy) and not inQueue[dx][dy]) {
            q.push({dx, dy, step + 1});
            inQueue[dx][dy] = true;
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

  // return berth_id, -1 if {x,y} is not a berth
  static int getBerthId(uint32_t x, uint32_t y) {
    if (_grids[x][y]._type == Type::berth)
      return _grids[x][y]._berth_id;
    return -1;
  }

  // is connected to any berth
  static bool isConnected(uint32_t x, uint32_t y) {
    switch (_grids[x][y]._type) {
    case Type::space: {
      const auto &arr = _grids[x][y]._dis;
      auto minp = std::min_element(std::begin(arr), std::end(arr));
      return *minp != static_cast<uint16_t>(-1);
    }
    case Type::berth:
      return true;
    case Type::ocean:
    case Type::barrier:
    default:
      return false;
    }
  }

  static bool isConnectedTo(uint32_t x, uint32_t y, uint32_t berth_id) {
    switch (_grids[x][y]._type) {
    case Type::space: {
      const auto &arr = _grids[x][y]._dis;
      return arr[berth_id] != static_cast<uint16_t>(-1);
    }
    case Type::berth: {
      uint32_t current_berth_id = getBerthId(x, y);
      assert(current_berth_id < BERTH_MAX);
      if (current_berth_id == berth_id)
        return true;
      auto x = berths[current_berth_id]._x;
      auto y = berths[current_berth_id]._y;
      for (int i = -1; i <= 4; i++) {
        for (int j = -1; j <= 4; j++) {
          auto nx = x + i;
          auto ny = y + j;
          if (not isMoveAble(nx, ny) or _grids[nx][ny]._type == Type::berth)
            continue;
          if (isConnectedTo(nx, ny, berth_id))
            return true;
        }
      }
      return false;
    }
    case Type::ocean:
    case Type::barrier:
    default:
      return false;
    }
  }

  // _grid[x][y] should be space , otherwise return 0
  static uint32_t countConnectedBerth(uint32_t x, uint32_t y) {
    const auto &grid = _grids[x][y];
    switch (grid._type) {
    case Type::space: {
      const auto &arr = grid._dis;
      return std::count_if(std::begin(arr), std::end(arr), [](uint16_t i) {
        assert(i <= MAP_X_AXIS_MAX * MAP_Y_AXIS_MAX);
        return i != static_cast<uint16_t>(-1);
      });
      break;
    }
    default:
      break;
    }
    return 0;
  }

  // return Nearest Berth <distance,id>
  static std::pair<uint32_t, uint32_t> nearestBerth(uint32_t x, uint32_t y) {
    const auto &grid = _grids[x][y];
    switch (grid._type) {
    case Type::berth:
      return {0, getBerthId(x, y)};
    case Type::space: {
      const auto &arr = grid._dis;
      auto minp = std::min_element(std::begin(arr), std::end(arr));
      if (*minp == static_cast<uint16_t>(-1))
        return {-1, -1};
      const auto id = std::distance(std::begin(arr), minp);
      return {arr[id], id};
    }
    default:
      return {-1, -1};
    }
  }

  // <berthid, distance（地图上最短距离）> [x,y] should be space
  static std::vector<std::pair<uint32_t, uint32_t>> connectedBerth(uint32_t x,
                                                                   uint32_t y) {
    std::vector<std::pair<uint32_t, uint32_t>> res;
    const auto &grid = _grids[x][y];
    switch (grid._type) {
    case Type::berth:
      res.push_back({getBerthId(x, y), 0});
      break;
    case Type::space: {
      for (uint32_t i = 0; i < BERTH_MAX; i++) {
        if (grid._dis[i] != static_cast<uint16_t>(-1))
          res.push_back({i, grid._dis[i]});
      }
    }
    default:
      break;
    }
    return res;
  }

  static uint32_t manhattanDistance(const std::pair<int, int> &a,
                                    const std::pair<int, int> &b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
  }

  // {Next step, distance}
  static std::pair<Direction, uint32_t>
  aStarSearch(const std::pair<uint32_t, uint32_t> from,
              const std::pair<uint32_t, uint32_t> to) {
    TimeCounter _tc{__FUNCTION__};
    // Solving ERR: Reference to local variable declared in enclosing function
    // static const auto _from = from;
    static const auto _to = to;

    // std::vector<Direction> pathd;
    /*
    对于任意一个格子n，其估价函数如下：
    f(n) = g(n) + h(n)
    其中 g(n) 指的是从起始格子到格子n的实际代价，
    而 h(n) 指的是从格子n到终点格子的估计代价。
    */

    struct gridPqItem {
      uint32_t _x, _y;
      uint32_t _steps; // steps from start grid

      // For priority queue , the smaller one is in the front
      bool operator<(const gridPqItem &rhs) const {
        return this->operator>(rhs);
      }

      bool operator>(const gridPqItem &rhs) const {

        const auto lhs_g = _steps;
        const auto lhs_h = manhattanDistance({_x, _y}, _to);
        const auto lhs_f = lhs_g + lhs_h;

        const auto rhs_g = rhs._steps;
        const auto rhs_h = manhattanDistance({rhs._x, rhs._y}, _to);
        const auto rhs_f = rhs_g + rhs_h;

        return std::tie(lhs_f, lhs_h) > std::tie(rhs_f, rhs_h);
      }
    };

    static bool inQueue[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
    memset(inQueue, 0, sizeof(inQueue));

    static Direction road[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
    std::fill_n(std::addressof(road[0][0]), MAP_X_AXIS_MAX * MAP_Y_AXIS_MAX,
                Direction::none);

    std::priority_queue<gridPqItem> pq;
    pq.push({from.first, from.second, 0});

    inQueue[from.first][from.second] = true;

    while (not pq.empty()) {
      auto [x, y, step] = pq.top();
      pq.pop();

      assert(inQueue[x][y]);

      if (x == to.first and y == to.second) {
        // backtrack
        uint32_t cnt = 0;
        Direction last = Direction::none;
        while (not(x == from.first and y == from.second)) {
          int idx = static_cast<int>(road[x][y]);
          // pathd.push_back(reverse(road[x][y]));
          last = reverse(road[x][y]);
          assert(road[x][y] != Direction::none);
          assert(idx < 4);
          x += _sdir[idx][0];
          y += _sdir[idx][1];
          assert(isMoveAble(x, y));
          cnt++;
        }
        // step is the shortest path length
        // if (step != pathd.size()) {
        //   for (auto &&dir : pathd) {
        //     std::cerr << "pathd: " << static_cast<int>(dir) << std::endl;
        //   }
        //   std::cerr << step << " " << pathd.size() << std::endl;
        //   assert(step == pathd.size());
        // }
        return {last, cnt};
      }

      for (int i = 0; i < 4; i++) {
        uint32_t nx = x + _sdir[i][0];
        uint32_t ny = y + _sdir[i][1];
        if (isMoveAble(nx, ny) and not inQueue[nx][ny]) {
          assert(road[nx][ny] == Direction::none);
          road[nx][ny] = reverse(static_cast<Direction>(i)); //! Maybe bug here
          pq.push({nx, ny, step + 1});
          assert(inQueue[nx][ny] == false);
          inQueue[nx][ny] = true;
        }
      }
    }
    // std::reverse(pathd.begin(), pathd.end());
    // return pathd;
    return {Direction::none, -1};
  }

  static const std::pair<uint32_t, uint32_t>
  getBerthPosition(uint32_t bid, bool random = false) {
    assert(bid < BERTH_MAX);
    const auto &bert = berths[bid];
    if (not random)
      return {bert._x, bert._y};

    const int xalias = getRandom(0, 3);
    const int yalias = getRandom(0, 3);
    return {bert._x + xalias, bert._y + yalias};
  }

} map;

char Map::_rawmap[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
Map::Grid Map::_grids[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];

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

  int _berth_id; // 目标泊位是虚拟点,则为-1

  uint32_t _size;

  uint32_t _free_frame; // 有多少frame , 该船没有装货物了

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

std::array<Ship, SHIP_MAX> ships;

bool getOutOfMyWay(std::vector<uint8_t> &boss, uint32_t rid);
uint32_t isThereARobot(uint32_t x, uint32_t y);

class Robot {
public:
  uint32_t _id;
  uint32_t _x, _y;

  enum class Status {
    recovering = 0,
    normal = 1,
    useless, // 位于的区域封闭,与码头不连通
  } _status;

  uint32_t _carry_cargo_id; // -1 表示机器人尚未装货, 或则表示装着 [id] 货物
  // uint32_t _target_cargo_id;

  bool _already_moved;

  bool isWithCargo() const {
    return _carry_cargo_id != static_cast<uint32_t>(-1);
  }

  bool isInBerth() const {
    return Map::_grids[_x][_y]._type == Map::Type::berth;
  }

  bool move(Direction direction) {
    if (direction == Direction::none or _already_moved)
      return false;

    uint32_t tmp = static_cast<uint32_t>(direction);
    assert(tmp < 4);

    uint32_t dx = _x + Map::_sdir[tmp][0];
    uint32_t dy = _y + Map::_sdir[tmp][1];

    assert(Map::isMoveAble(dx, dy));
    int rid = isThereARobot(dx, dy);
    if (rid != -1) {
      std::vector<uint8_t> boss;
      boss.push_back(this->_id);
      auto flag = getOutOfMyWay(boss, rid);
      if (not flag)
        return false;
    }

    _x = dx;
    _y = dy;
    _already_moved = true;

    assert(_id < ROBOT_MAX);

    // printf("move %u %u\n", _id, static_cast<uint32_t>(direction));
    robots_actions.push_back(
        {RobotAction::ActionType::move,
         {static_cast<uint8_t>(_id), static_cast<uint8_t>(direction)}});

    return true;
  }

  void get() {
    assert(_id < ROBOT_MAX);
    // printf("get %u\n", _id);
    robots_actions.push_back(
        {RobotAction::ActionType::get, {static_cast<uint8_t>(_id), 0}});
  }
  void pull() {
    int bid = Map::getBerthId(_x, _y);
    assert(bid != -1);
    assert(berths[bid]._cargo_ongoing > 0);
    berths[bid]._cargo_ongoing--;
    berths[bid]._cargoIds.push_back(_carry_cargo_id);
    _carry_cargo_id = -1;

    // printf("pull %u\n", _id);
    assert(_id < ROBOT_MAX);
    robots_actions.push_back(
        {RobotAction::ActionType::pull, {static_cast<uint8_t>(_id), 0}});
  }
};

std::array<Robot, ROBOT_MAX> robots;

uint32_t isThereARobot(uint32_t x, uint32_t y) {
  auto p = std::find_if(begin(robots), end(robots), [x, y](const Robot &rob) {
    return rob._x == x and rob._y == y;
  });
  if (p != end(robots))
    return p->_id;
  return -1;
}

// return true if rob[rid] can move away , false if rob[rid] is already moved or
// no way to move.
bool getOutOfMyWay(std::vector<uint8_t> &boss, uint32_t rid) {
  if (robots[rid]._already_moved) {
    boss.pop_back();
    return false;
  }
  const auto &dirs = getRandomDir();
  const auto x = robots[rid]._x, y = robots[rid]._y;

  std::vector<uint8_t> blocked_robs;

  for (auto &&dir : dirs) {
    if (Direction::none == static_cast<Direction>(dir))
      continue;
    auto dx = x + Map::_sdir[dir][0], dy = y + Map::_sdir[dir][1];
    if (not Map::isMoveAble(dx, dy))
      continue;
    int resid = isThereARobot(dx, dy);
    if (resid == -1) { // 没有机器人,可以走
      boss.pop_back();
      return robots[rid].move(
          static_cast<Direction>(dir)); // should always be true
    } else if (std::find(boss.begin(), boss.end(), resid) == boss.end())
      blocked_robs.push_back(resid);
  }
  for (auto bid : blocked_robs) {
    boss.push_back(rid);
    bool flag = getOutOfMyWay(boss, bid);
    if (flag) {
      boss.pop_back();
      return true;
    }
  }
  boss.pop_back();
  return false;
}

class Cargo {
public:
  uint32_t _id;
  uint32_t _x, _y;
  uint32_t _price; // 货物的价值
  uint32_t _disappear_frame;
  bool _taken; // 被机器人已经拿起
};
std::vector<Cargo> cargos;

class CargoPqItem {
public:
  uint32_t _id;
  CargoPqItem() = delete;
  CargoPqItem(uint32_t id) : _id(id) {}
  CargoPqItem(const Cargo &cargo) : _id(cargo._id) {}
  CargoPqItem(const CargoPqItem &item) : _id(item._id) {}

  // <manhattan,robotsid> if {sorted} is true, sorted by manhattan distance
  static std::vector<std::pair<uint32_t, uint32_t>>
  manhattanRobots(uint32_t cargo_id, bool sorted = true) {
    std::vector<std::pair<uint32_t, uint32_t>> ret;
    auto cx = cargos[cargo_id]._x, cy = cargos[cargo_id]._y;
    for (auto &&robot : robots) {
      if (robot._status == Robot::Status::useless or
          robot._carry_cargo_id != static_cast<uint32_t>(-1))
        continue;
      auto manha = Map::manhattanDistance({cx, cy}, {robot._x, robot._y});
      ret.push_back({manha, robot._id});
    }
    if (sorted)
      std::sort(begin(ret), end(ret));
    return ret;
  }

  // <manhattan distance, robot id>
  static std::pair<uint32_t, uint32_t>
  nearestRobotByManhattan(uint32_t cargo_id) {
    auto &&manha = CargoPqItem::manhattanRobots(cargo_id, false);
    if (manha.empty())
      return {-1, -1};
    const auto &p = std::min_element(begin(manha), end(manha));
    return *p;
  }

  static float averageManhattanRob(const uint32_t cargo_id,
                                   uint32_t iter_max = 3) {
    assert(cargo_id < cargos.size());
    const auto &rs = manhattanRobots(cargo_id);
    uint32_t cnt = 0;
    uint32_t manh_sum = 0;
    for (; cnt < rs.size() and cnt < iter_max; ++cnt) {
      const auto [manh, rid] = rs[cnt];
      manh_sum += manh;
    }
    if (cnt == 0)
      return std::numeric_limits<float>::max();
    return static_cast<float>(manh_sum) / cnt;
  }

  bool operator<(const CargoPqItem &rhs) const {
    assert(_id < cargos.size());
    assert(rhs._id < cargos.size());

    const auto &lhs_cargo = cargos[this->_id];
    const auto &rhs_cargo = cargos[rhs._id];

    const int lhs_flag = lhs_cargo._taken ? 1 : 0;
    const int rhs_flag = rhs_cargo._taken ? 1 : 0;
    if (lhs_flag or rhs_flag)
      return lhs_flag > rhs_flag;

    int32_t lhs_remain_frame = lhs_cargo._disappear_frame - getCurrentFrame();
    int32_t rhs_remain_frame = rhs_cargo._disappear_frame - getCurrentFrame();
    if (lhs_remain_frame <= 0 or rhs_remain_frame <= 0)
      return lhs_remain_frame < rhs_remain_frame;

    const auto [lhs_nb_dis, lhs_nb_id] =
        Map::nearestBerth(lhs_cargo._x, lhs_cargo._y);
    const auto [rhs_nb_dis, rhs_nb_id] =
        Map::nearestBerth(rhs_cargo._x, rhs_cargo._y);

    const float lhs_avg_rob = averageManhattanRob(lhs_cargo._id);
    const float rhs_avg_rob = averageManhattanRob(rhs_cargo._id);

    lhs_remain_frame -= (lhs_nb_dis + lhs_avg_rob);
    rhs_remain_frame -= (rhs_nb_dis + rhs_avg_rob);

    if (lhs_remain_frame <= 0 or rhs_remain_frame <= 0)
      return lhs_remain_frame < rhs_remain_frame;

    const float lhs_price = lhs_cargo._price;
    const float rhs_price = rhs_cargo._price;

    const float lhs_score = lhs_price / (lhs_nb_dis + lhs_avg_rob);
    const float rhs_score = rhs_price / (rhs_nb_dis + rhs_avg_rob);

    return lhs_score < rhs_score;
  }

  bool operator==(const CargoPqItem &rhs) const { return _id == rhs._id; }
  bool operator>(const CargoPqItem &rhs) const {
    if (*this == rhs)
      return false;
    return rhs < *this;
  }
};

std::priority_queue<CargoPqItem> cargos_pq;
// std::vector<CargoPqItem> cargo_pqs;

void initialization() {
  cargos.reserve(10 * FRAME_MAX);
  robots_actions.reserve(ROBOT_MAX);
  ships_actions.reserve(SHIP_MAX);

  map.readmap();

  for (uint32_t i = 0; i < BERTH_MAX; i++) {
    uint32_t id, x, y, time, velocity;
    // time(1 <= time <=
    // 1000)表示该泊位轮船运输到虚拟点的时间(虚拟点移动到泊位的时间同),即产生价值的时间,时间用帧数表示。
    // Velocity(1 <= Velocity <=
    // 5)表示该泊位的装载速度,即每帧可以装载的物品数,单位是:个。
    scanf("%u %u %u %u %u", &id, &x, &y, &time, &velocity);
    // berths[i] = {id, x, y, time, velocity};
    berths[id]._id = id;
    berths[id]._x = x;
    berths[id]._y = y;
    berths[id]._time = time;
    berths[id]._velocity = velocity;
    berths[id]._cargo_ongoing = 0;

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

  assert(frame_id == frame_current);

  frame_current = frame_id;

  uint32_t K;
  // 场上新增货物的数量 K
  scanf("%u", &K);
  for (uint32_t i = 0; i < K; ++i) {
    uint32_t x, y, price;
    // 货物的位置坐标、金额
    scanf("%u %u %u", &x, &y, &price);
    if (not Map::isConnected(x, y)) // 货物在封闭区域,忽略之
      continue;

    cargos.push_back({static_cast<uint32_t>(cargos.size()), x, y, price,
                      frame_id + FRAME_CARGO_REMAIN, false});
    cargos_pq.push(cargos.back());
    // cargo_pqs.push_back(cargos.back());
    auto [dis, bid] = Map::nearestBerth(x, y);
    berths[bid]._cargo_ongoing++;
  }

  for (uint8_t id = 0; id < ROBOT_MAX; ++id) {
    uint32_t carryflag, x, y, status;
    // carryflag: 0 表示未携带物品,1 表示携带物品。
    // x,y 机器人所在位置坐标
    // status:  0 表示恢复状态,1 表示正常运行状态
    scanf("%u %u %u %u", &carryflag, &x, &y, &status);
    robots[id]._id = id;
    robots[id]._x = x;
    robots[id]._y = y;
    robots[id]._already_moved = false;

    if (not Map::isConnected(x, y))
      robots[id]._status = Robot::Status::useless;
    else
      robots[id]._status = static_cast<Robot::Status>(status);

    if (carryflag == 0) {
      robots[id]._carry_cargo_id = -1;
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
    ships[id]._berth_id = berth_id;
    if (status == 1 and berth_id == -1) {
      ships[id]._size = 0;
      ships[id]._free_frame = 0;
    }
  }

  char okk[32];
  scanf("%s", okk);
  return frame_id;
};

class CargoRobotDispatcher {
  std::unordered_set<uint32_t> _unavaiable_robs;
  // key <rob_id,cargo_id> value <Direction>
  std::map<std::pair<uint32_t, uint32_t>, Direction> _robs_act;

public:
  CargoRobotDispatcher() {
    for (uint32_t i = 0; i < ROBOT_MAX; ++i) {
      const auto &rob = robots[i];
      if (rob.isWithCargo() or rob._status != Robot::Status::normal)
        _unavaiable_robs.insert(i);
    }
  }
  CargoRobotDispatcher(const CargoRobotDispatcher &rhs) = delete;
  uint32_t countAvaiableRobots() const {
    assert(_unavaiable_robs.size() <= ROBOT_MAX);
    return ROBOT_MAX - _unavaiable_robs.size();
  }

  uint32_t dispatch(uint32_t cargo_id, uint32_t bestof = 3) {
    if (cargos[cargo_id]._taken)
      return -1;
    std::vector<std::pair<uint32_t, uint32_t>> res; // {distance, robot_id}
    const auto &manh_robs = CargoPqItem::manhattanRobots(cargo_id);
    for (uint32_t i = 0; i < bestof and i < manh_robs.size(); ++i) {
      const auto rid = manh_robs[i].second;
      if (manh_robs[i].first == 0) { // 直接到达
        res.push_back({0, rid});
        _robs_act[{rid, cargo_id}] = Direction::none;
        break;
      }
      if (_unavaiable_robs.count(
              rid)) // Robot already carry cargo or not available
        continue;
      auto &&astar =
          Map::aStarSearch({robots[rid]._x, robots[rid]._y},
                           {cargos[cargo_id]._x, cargos[cargo_id]._y});
      res.push_back({astar.second, rid});
      _robs_act[{rid, cargo_id}] = astar.first;
    }
    if (not res.empty()) {
      auto minp = std::min_element(begin(res), end(res));
      _unavaiable_robs.insert(minp->second);
      return minp->second;
    }
    return -1;
  }

  Direction getDirection(uint32_t rob_id, uint32_t cargo_id) const {
    auto it = _robs_act.find({rob_id, cargo_id});
    if (it == end(_robs_act))
      return Direction::none;
    return it->second;
  }
};

class BerthPqItem {
public:
  uint32_t _bid;
  uint32_t _tillu;

  BerthPqItem(uint32_t bid) : _bid(bid) {
    const auto &lhs = berths[_bid];
    const auto lhs_wait_ships = lhs._shipIds.size();

    auto lhs_cap = Ship::capacity * lhs_wait_ships;
    if (lhs_wait_ships > 0) {
      auto sh_nid = lhs._shipIds.front().second;
      lhs_cap -= ships[sh_nid]._size;
    }

    _tillu = lhs_cap / lhs._velocity;
  }

  bool operator<(const BerthPqItem &rhsitem) const {
    const auto &lhs = berths[_bid];
    const auto &rhs = berths[rhsitem._bid];

    const auto lhs_wait_ships = lhs._shipIds.size();
    const auto rhs_wait_ships = rhs._shipIds.size();

    auto lhs_cap = Ship::capacity * lhs_wait_ships;
    if (lhs_wait_ships > 0) {
      auto sh_nid = lhs._shipIds.front().second;
      lhs_cap -= ships[sh_nid]._size;
    }

    auto rhs_cap = Ship::capacity * rhs_wait_ships;
    if (rhs_wait_ships > 0) {
      auto sh_nid = rhs._shipIds.front().second;
      rhs_cap -= ships[sh_nid]._size;
    }

    const auto lhs_cargo_num = lhs._cargo_ongoing + lhs._cargoIds.size();
    const auto rhs_cargo_num = rhs._cargo_ongoing + rhs._cargoIds.size();

    const auto lhs_timecost = lhs._time;
    const auto rhs_timecost = rhs._time;

    const auto lhs_untilme = lhs_cap / lhs._velocity;
    const auto rhs_untilme = rhs_cap / rhs._velocity;

    int lhs_wait_frame = lhs_untilme - lhs_timecost;
    int rhs_wait_frame = rhs_untilme - rhs_timecost;

    const float lhs_remain_cargos = lhs_cargo_num - lhs_cap;
    const float rhs_remain_cargos = rhs_cargo_num - rhs_cap;

    const float lhs_score =
        lhs_remain_cargos /
        (lhs_wait_frame + (lhs_remain_cargos) / lhs._velocity + lhs._time);

    const float rhs_score =
        rhs_remain_cargos /
        (rhs_wait_frame + (rhs_remain_cargos) / rhs._velocity + rhs._time);

    return lhs_score < rhs_score; // ! maybe BUG BUG here
  }
};

void frameUpdate() {
  // TimeCounter _tc{__FUNCTION__};
  // std::sort(rbegin(cargo_pqs), rend(cargo_pqs));

  // while (not cargo_pqs.empty()) {
  //   const auto back = cargo_pqs.back();
  //   bool flag = cargos[back._id]._taken;
  //   int rem = cargos[back._id]._disappear_frame - getCurrentFrame();
  //   if (rem <= 0)
  //     flag = true;
  //   if (flag)
  //     cargo_pqs.pop_back();
  //   else
  //     break;
  // }

  CargoRobotDispatcher dispatcher;
  int avaiable_rots = dispatcher.countAvaiableRobots();

  std::vector<CargoPqItem> cargos_tmp;

  while (not cargos_pq.empty() and avaiable_rots > 0) {
    // for (uint32_t i = 0; i < cargo_pqs.size() and avaiable_rots > 0; ++i) {
    // const auto &cargo = cargos[cargo_pqs[i]._id];
    const auto cargoitem = cargos_pq.top();
    cargos_pq.pop();

    const auto &cargo = cargos[cargoitem._id];
    if (cargo._taken)
      continue;
    if (cargo._disappear_frame <= getCurrentFrame())
      continue;
    cargos_tmp.push_back(cargoitem);
    int rob_id = dispatcher.dispatch(cargo._id);
    if (rob_id == -1)
      continue;

    avaiable_rots--;
    auto &rob = robots[rob_id];
    // let rob_id goto cargo._id
    auto dir = dispatcher.getDirection(rob_id, cargo._id);
    if (dir == Direction::none) {
      cargos_tmp.pop_back();
      // std::cerr << rob_id << std::endl;
      rob.get();
      rob._carry_cargo_id = cargo._id;
      cargos[cargo._id]._taken = true; // 写法不够优雅
    } else {
      rob.move(dir);
    }
  }

  for (auto &&item : cargos_tmp) {
    cargos_pq.push(item);
  }

  // Now let robots which is with cargo moving
  for (auto &&rob : robots) {
    if (rob._status != Robot::Status::normal)
      continue;
    if (not rob.isWithCargo())
      continue;
    if (rob.isInBerth()) {
      rob.pull();
    }
    auto [dis, bid] = Map::nearestBerth(rob._x, rob._y);
    if (dis == 0) {
      rob.pull();
    } else {
      auto &&dirs =
          Map::aStarSearch({rob._x, rob._y}, Map::getBerthPosition(bid, true));
      auto dir = dirs.first;
      rob.move(dir);
    }
  }

  std::vector<BerthPqItem> berth_pqs;

  for (auto &&b : berths) {
    // the back one is the current one
    std::sort(std::rbegin(b._shipIds), rend(b._shipIds));
    berth_pqs.emplace_back(b._id);
  }

  std::sort(begin(berth_pqs), end(berth_pqs));

  // Ship action
  for (auto &&sh : ships) {
    if (sh._status != Ship::Status::normal)
      continue;
    if (sh._berth_id == -1) { // 处于虚拟点
      if (berth_pqs.empty())
        continue;
      auto bid = berth_pqs.back()._bid;
      auto tillu = berth_pqs.back()._tillu;
      const auto &bert = berths[bid];

      if (tillu <= bert._time) {
        sh.ship(bid);
        berths[bid]._shipIds.push_back(
            {getCurrentFrame() + berths[bid]._time, sh._id});
      }

      berth_pqs.pop_back();

    } else { // 处于泊位

      auto &&bert = berths[sh._berth_id];
      uint32_t onoboard = 0;
      while (not bert._cargoIds.empty() and onoboard < bert._velocity) {
        if (sh._size < Ship::capacity) {
          bert._cargoIds.pop_front();
          sh._size++;
          onoboard++;
        } else {
          break;
        }
      }
      if (onoboard == 0)
        sh._free_frame++;
      else
        sh._free_frame = 0;

      if (sh._size == Ship::capacity or
          getCurrentFrame() + bert._time + 1 >= FRAME_MAX) { // 发船
        sh.go();
        assert(bert._shipIds.front().second == sh._id);
        bert._shipIds.pop_front();
      } else if (sh._free_frame > 25) {
        bert._shipIds.pop_front();
        if (bert._time * 2 < FRAME_SHIP_SWITH_FROM_BERTH) { // 回虚拟点
          sh.go();
          assert(bert._shipIds.front().second == sh._id);
        } else { // 转港口
          int bid = rand() % BERTH_MAX;
          sh.ship(bid);
          berths[bid]._shipIds.push_back(
              {getCurrentFrame() + FRAME_SHIP_SWITH_FROM_BERTH, sh._id});
        }
      }
    }
  }
}

void frameOutput() {
  for (auto &&action : robots_actions) {
    assert(action._param[0] < ROBOT_MAX);
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
    auto p = freopen(debugPath, "r", stdin);
    assert(p);
    *p = *p;
  }
#endif

  initialization();

  for (; frame_current <= FRAME_MAX; ++frame_current) {
    auto ret = frameInput();
    assert(ret == frame_current);
    ret = ret;

    frameUpdate();

    frameOutput();
    frameDone();
  }
  return 0;
}