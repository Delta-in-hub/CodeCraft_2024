#include "def.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <sys/types.h>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

uint32_t frame_current = 1;

uint32_t all_values_get, all_values_pull;

std::array<uint32_t, BERTH_MAX> berth_govp_via;

uint32_t getCurrentFrame() { return frame_current; }

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
    // if (duration >= std::chrono::milliseconds(1))
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

// Order is important!
class Cargo {
public:
  uint32_t _id;
  uint32_t _origin_x, _origin_y;
  uint32_t _price; // 货物的价值
  uint32_t _disappear_frame;
  bool _taken;        // 已经被机器人拿起
  uint32_t _berth_id; // 距离最近的港口id
  float _score;

  bool isAvailable() const {
    if (getCurrentFrame() >= _disappear_frame)
      return false;
    if (_taken)
      return false;
    return true;
  }
};
std::vector<Cargo> cargos;

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
  uint32_t _values;

  uint32_t _free_frame; // 累计有多少frame , 该船没有装货物了

  void clear() {
    _size = 0;
    _values = 0;
    _free_frame = 0;
  }

  void ship(uint32_t berth_target_id) {
    // printf("ship %u %u\n", _id, berth_id);
    assert(berth_target_id < BERTH_MAX);

    this->_berth_id = berth_target_id;
    this->_free_frame = 0;

    ships_actions.push_back(
        {ShipAction::ActionType::ship,
         {static_cast<uint8_t>(_id), static_cast<uint8_t>(berth_target_id)}});
  }
  void go() {
    // printf("go %u\n", _id);
    this->_berth_id = -1;
    this->_free_frame = 0;
    this->clear();

    ships_actions.push_back(
        {ShipAction::ActionType::go, {static_cast<uint8_t>(_id), 0}});
  }
};

uint32_t Ship::capacity = -1;

std::array<Ship, SHIP_MAX> ships;

class Berth {
public:
  uint32_t _id;
  uint32_t _x, _y;
  uint32_t _time;     // 该泊位轮船运输到虚拟点的时间
  uint32_t _velocity; // 该泊位的装载速度
  // uint32_t _cargo_ongoing;
  std::deque<std::pair<uint32_t, uint32_t>>
      _ships_here; // 该泊位的船只; {arrive_frame,ship id}
  std::deque<std::pair<uint32_t, uint32_t>>
      _cargos_here; // 该泊位的货物; {price , cargo id}

  std::deque<std::tuple<uint32_t, uint32_t, uint32_t>>
      _ships_via; // 借路该泊位去 target ; {arrive_frame,ship id,target id}

  // <arrive_frame,ship id> Ship arrives here
  void loadShip(std::pair<uint32_t, uint32_t> sh) { _ships_here.push_back(sh); }

  // {price,cargo id}
  void loadCargo(std::pair<uint32_t, uint32_t> cg) {
    _cargos_here.push_back(cg);
  }

  void leaveShip() {
    if (_ships_here.empty())
      return;
    _ships_here.pop_front();
  }

  uint32_t cargoGoOnBoardInOneFrame() {
    if (_ships_here.empty())
      return 0;
    const auto p = _ships_here.front();
    if (getCurrentFrame() < p.first)
      return 0;

    auto &ship = ships[p.second];
    if (ship._status != Ship::Status::normal)
      return 0;

    uint32_t cargos_cnt = _cargos_here.size();
    uint32_t can = std::min(cargos_cnt, Ship::capacity - ship._size);

    for (uint32_t i = 0; i < can; i++) {
      const auto p = _cargos_here.front();
      _cargos_here.pop_front();
      ship._size++;
      ship._values += p.first;
    }

    return can;
  }

  auto curShip() const { return _ships_here.front(); }

  uint32_t countShipsNow() const {
    auto ret = std::count_if(begin(_ships_here), end(_ships_here),
                             [](const std::pair<uint32_t, uint32_t> p) {
                               return getCurrentFrame() >= p.first;
                             });
    return ret;
  }
  uint32_t countShips() const { return _ships_here.size(); }
  uint32_t countCargosNow() const { return _cargos_here.size(); }
  uint32_t sumCargosValueNow() const {
    uint32_t ret = std::accumulate(
        begin(_cargos_here), end(_cargos_here), 0,
        [](uint32_t sum, const std::pair<uint32_t, uint32_t> p) {
          return sum + p.first;
        });
    return ret;
  }
};
std::array<Berth, BERTH_MAX> berths;

uint32_t goVpNeedTime(uint32_t _bid) {
  auto viabid = berth_govp_via[_bid];
  if (viabid == _bid)
    return berths[_bid]._time;
  return FRAME_SHIP_SWITH_FROM_BERTH + berths[viabid]._time;
}

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

  static bool isOutOfRange(uint32_t x, uint32_t y) {
    if (unlikely(x >= MAP_X_AXIS_MAX or y >= MAP_Y_AXIS_MAX))
      return true;
    return false;
  }

  static bool isMoveAble(uint32_t x, uint32_t y) {
    if (isOutOfRange(x, y))
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
    if (unlikely(_grids[x][y]._type == Type::berth))
      return _grids[x][y]._berth_id;
    return -1;
  }

  // is connected to any berth
  static bool isConnectedToAnyBerth(uint32_t x, uint32_t y) {
    switch (_grids[x][y]._type) {
    case Type::space: {
      const auto &arr = _grids[x][y]._dis;
      auto minp = std::min_element(std::begin(arr), std::end(arr));
      return *minp != static_cast<uint16_t>(-1);
    }
    case Type::berth:
    case Type::ocean:
    case Type::barrier:
      return true;
    default:
      return false;
    }
  }

  // Does {x,y} is connected to berth[id]
  static bool isConnectedToBerth(uint32_t x, uint32_t y, uint32_t berth_id) {
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
          if (isConnectedToBerth(nx, ny, berth_id))
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
        return i != static_cast<uint16_t>(-1);
      });
      break;
    }
    default:
      break;
    }
    return 0;
  }

  static uint32_t getDistanceToBerth(uint32_t x, uint32_t y, uint32_t bid) {
    switch (_grids[x][y]._type) {
    case Map::Type::space:
      return _grids[x][y]._dis[bid];
    case Map::Type::berth: {
      uint32_t nowbid = Map::getBerthId(x, y);
      if (nowbid == bid)
        return 0;
      else {
        auto x = berths[nowbid]._x;
        auto y = berths[nowbid]._y;
        for (int i = -1; i <= 4; i++) {
          for (int j = -1; j <= 4; j++) {
            auto nx = x + i;
            auto ny = y + j;
            if (not isMoveAble(nx, ny) or _grids[nx][ny]._type == Type::berth)
              continue;
            int ret = getDistanceToBerth(nx, ny, bid);
            if (ret != -1)
              return ret;
          }
        }
        return -1;
      }
    }
    default:
      return -1;
    }
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

  // 判断两个格子是否连通
  static bool isConnected(const std::pair<uint32_t, uint32_t> from,
                          const std::pair<uint32_t, uint32_t> to) {
    const auto [fx, fy] = from;
    const auto [tx, ty] = to;

    // bugs

    auto ft = _grids[fx][fy]._type;
    auto tt = _grids[tx][ty]._type;

    if (ft == Map::Type::berth) {
      auto fbid = _grids[fx][fy]._berth_id;
      bool flag = Map::isConnectedToBerth(tx, ty, fbid);
      return flag;
    }

    if (tt == Map::Type::berth) {
      auto tbid = _grids[tx][ty]._berth_id;
      bool flag = Map::isConnectedToBerth(fx, fy, tbid);
      return flag;
    }

    const auto &fdis = _grids[fx][fy]._dis;
    const auto &tdis = _grids[tx][ty]._dis;
    for (uint32_t i = 0; i < BERTH_MAX; i++) {
      if (fdis[i] != static_cast<uint16_t>(-1) and
          tdis[i] != static_cast<uint16_t>(-1)) {
        return true;
      }
    }
    return false;
  }

  static uint32_t manhattanDistance(const std::pair<int, int> a,
                                    const std::pair<int, int> b) {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
  }

  // {Next step, distance}
  static std::pair<Direction, uint32_t>
  aStarSearch(const std::pair<uint32_t, uint32_t> from,
              const std::pair<uint32_t, uint32_t> to) {
    // TimeCounter _tc{__FUNCTION__};
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

    // sizeof(gridPqItem) == 8 , so pass by value
    struct gridPqItem {
      uint16_t _x, _y;
      uint32_t _steps; // steps from start grid

      // For priority queue , the smaller one is in the front
      bool operator<(const gridPqItem rhs) const {
        return this->operator>(rhs);
      }

      bool operator>(const gridPqItem rhs) const {

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
    // std::fill_n(std::addressof(road[0][0]), MAP_X_AXIS_MAX * MAP_Y_AXIS_MAX,
    //             Direction::none);

    static_assert(
        sizeof(Direction) == sizeof(char),
        "sizeof(Direction) should equals to sizeof(char) to use memest");
    memset(road, static_cast<int>(Direction::none), sizeof(road));

    std::priority_queue<gridPqItem> pq;
    pq.push({static_cast<uint16_t>(from.first),
             static_cast<uint16_t>(from.second), 0});

    inQueue[from.first][from.second] = true;

    while (likely(not pq.empty())) {
      auto [x, y, step] = pq.top();
      pq.pop();

      assert(inQueue[x][y]);

      if (unlikely(x == to.first and y == to.second)) {
        // backtrack
        uint32_t cnt = 0;
        Direction last = Direction::none;
        while (not(x == from.first and y == from.second)) {
          int idx = static_cast<int>(road[x][y]);
          // pathd.push_back(reverse(road[x][y]));
          last = reverse(road[x][y]);
          // assert(road[x][y] != Direction::none);
          // assert(idx < 4);
          x += _sdir[idx][0];
          y += _sdir[idx][1];
          cnt++;
        }
        assert(cnt == step);
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
        uint16_t nx = x + _sdir[i][0];
        uint16_t ny = y + _sdir[i][1];
        if (isOutOfRange(nx, ny))
          continue;
        if (inQueue[nx][ny])
          continue;

        if (isMoveAble(nx, ny)) {
          road[nx][ny] = reverse(static_cast<Direction>(i));
          assert(road[nx][ny] != Direction::none);
          pq.push({nx, ny, step + 1});
          // assert(inQueue[nx][ny] == false);
          inQueue[nx][ny] = true;
        } else if (to.first == nx and to.second == ny) {
          // 目标点被优先级高的机器人设为了barrier
          if (step > 1) {
            road[nx][ny] = reverse(static_cast<Direction>(i));
            pq.push({nx, ny, step + 1});
            inQueue[nx][ny] = true;
          } else {
            return {Direction::none, step + 1};
          }
        }
      }
    }
    // std::reverse(pathd.begin(), pathd.end());
    // return pathd;
    return {Direction::none, -1};
  }

  static const std::pair<uint32_t, uint32_t> getBerthPosition(uint32_t bid) {
    assert(bid < BERTH_MAX);
    const auto &bert = berths[bid];
    return {bert._x, bert._y};
  }

  static Direction getDirection(std::pair<uint32_t, uint32_t> old,
                                std::pair<uint32_t, uint32_t> newp) {
    if (old == newp)
      return Direction::none;
    for (int i = 0; i < 4; i++) {
      std::pair<uint32_t, uint32_t> np = old;
      np.first += _sdir[i][0];
      np.second += _sdir[i][1];
      if (np == newp)
        return static_cast<Direction>(i);
    }
    return Direction::none;
  }
};

char Map::_rawmap[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
Map::Grid Map::_grids[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];

// Order is IMPORTANT!
const int Map::_sdir[4][2] = {{0, 1},  // 右
                              {0, -1}, // 左
                              {-1, 0}, // 上
                              {1, 0}}; // 下

Map::Type map_old[MAP_X_AXIS_MAX][MAP_Y_AXIS_MAX];
static std::vector<std::pair<uint32_t, uint32_t>> robotsAsBarrier;

void setBarrier(uint32_t x, uint32_t y) {
  map_old[x][y] = Map::_grids[x][y]._type;

  Map::_grids[x][y]._type = Map::Type::barrier;
  robotsAsBarrier.push_back(std::make_pair(x, y));
}

void unsetAllBarriers() {
  for (auto [x, y] : robotsAsBarrier) {
    Map::_grids[x][y]._type = map_old[x][y];
  }
  robotsAsBarrier.clear();
}

std::pair<uint32_t, uint32_t> robots_originp[ROBOT_MAX];

class Robot {
public:
  uint32_t _id;
  uint32_t _x, _y;

  enum class Status {
    recovering = 0,
    normal = 1,
    useless, // 位于的区域封闭,与码头不连通
  } _status;

  enum class CarryStatus {
    empty = 0,
    carrying = 1,
  } _carryStatus;

  uint32_t _target_cargo_id; // 目标货物id

  uint32_t _carry_cargo_id; // -1 表示机器人尚未装货, 或则表示装着 [id] 货物

  float _score;

  static std::array<Robot, ROBOT_MAX> robots;

  static uint32_t isThereARobotNow(uint32_t x, uint32_t y) {
    auto cnt = std::find_if(
        begin(Robot::robots), end(Robot::robots),
        [x, y](const Robot &r) { return r._x == x and r._y == y; });
    if (likely(cnt == end(robots)))
      return -1;
    return cnt->_id;
  }

  bool isWithCargo() const { return _carryStatus == CarryStatus::carrying; }

  bool isInBerth() const {
    return Map::_grids[_x][_y]._type == Map::Type::berth;
  }

  bool canGetCargo() const {
    if (isWithCargo())
      return false;
    if (_target_cargo_id == static_cast<uint32_t>(-1))
      return false;
    const auto &c = cargos[_target_cargo_id];
    if (c._origin_x == _x and c._origin_y == _y)
      return true;
    else
      return false;
  }

  bool isGoingToBerth() const { return isWithCargo(); }
  bool isGoingToCargo() const { return !isWithCargo(); }

  // 随时随地可以pull
  bool pull() {
    // if (not isWithCargo())
    //   return;

    int bid = Map::getBerthId(_x, _y);
    if (bid == -1)
      return false;

    if (not isWithCargo())
      return false;

    assert(_carry_cargo_id != (uint32_t)-1);

    auto &bert = berths[bid];

    all_values_pull += cargos[_carry_cargo_id]._price;

    bert.loadCargo({cargos[_carry_cargo_id]._price, _carry_cargo_id});

    _carry_cargo_id = -1;
    _carryStatus = CarryStatus::empty;
    if (_target_cargo_id != static_cast<uint32_t>(-1)) {
      _score = cargos[_target_cargo_id]._score;
    } else {
      _score = 0;
    }

    // printf("pull %u\n", _id);
    assert(_id < ROBOT_MAX);
    robots_actions.push_back(
        {RobotAction::ActionType::pull, {static_cast<uint8_t>(_id), 0}});
    return true;
  }

  std::pair<uint32_t, uint32_t> getCurPos() const { return {_x, _y}; }

  uint32_t _robot_carry_cnt;

  std::pair<uint32_t, uint32_t> getNextPos() const {

    if (not isWithCargo() and _target_cargo_id == static_cast<uint32_t>(-1)) {
      return getCurPos(); // ! bug
    }

    uint32_t tx, ty;

    if (not isWithCargo()) {
      const auto &c = cargos[_target_cargo_id];
      tx = c._origin_x;
      ty = c._origin_y;
    } else {
      const auto &c = cargos[_carry_cargo_id];

      uint32_t _bid;

      if (getCurrentFrame() <= 1) {
        auto &&bers = Map::connectedBerth(this->_x, this->_y); // bid ,dis
        int idx = this->_id % bers.size();
        _bid = bers[idx].first;
      } else if (getCurrentFrame() + FRAME_LAST >= FRAME_MAX) {
        auto &&bers = Map::connectedBerth(this->_x, this->_y);
        // bid ,dis
        using P = std::pair<uint32_t, uint32_t>;
        std::sort(begin(bers), end(bers), [](const P p1, const P p2) {
          float lhs_re = 0.5, rhs_re = 0.5;
          uint32_t lhs_dis = p1.second + 1;
          uint32_t rhs_dis = p2.second + 1;

          {
            auto &&bert = berths[p1.first];
            if (bert.countShipsNow()) {
              auto lhs_sid = bert.curShip().second;
              auto &&lhs_s = ships[lhs_sid];
              lhs_re = Ship::capacity - lhs_s._size;
            }
          }
          {
            auto &&bert = berths[p2.first];
            if (bert.countShipsNow()) {
              const auto rhs_sid = bert.curShip().second;
              const auto &rhs_s = ships[rhs_sid];
              rhs_re = Ship::capacity - rhs_s._size;
            }
          }
          float lhs_sc = (float)lhs_re / lhs_dis;
          float rhs_sc = (float)rhs_re / rhs_dis;
          return lhs_sc < rhs_sc;
        });
        _bid = bers.back().first;
      }
      // if (getCurrentFrame() + 3000 >= FRAME_MAX) {
      //   auto &&bers = Map::connectedBerth(this->_x, this->_y); // bid ,dis
      //   int idx = this->_id % bers.size();
      //   idx %= SHIP_MAX;
      //   _bid = bers[idx].first;
      // } else {
      // if (_robot_carry_cnt % 1000 == 0) {
      //   auto &&bers = Map::connectedBerth(this->_x, this->_y); // bid ,dis
      //   int idx = this->_id % bers.size();
      //   _bid = bers[idx].first;
      else {
        auto [_dis, ___bid] = Map::nearestBerth(this->_x, this->_y);
        _bid = ___bid;
      }
      // }

      auto [___x, ___y] = Map::getBerthPosition(_bid);

      tx = ___x + ((c._id / 4) % 4);
      ty = ___y + ((c._id % 4));
    }

    auto [direction, distance] = Map::aStarSearch({_x, _y}, {tx, ty});

    if (direction == Direction::none) { // 无路可走
      std::cerr << this->_id << ' ' << _x << ' ' << _y;
      std::cerr << ' ' << tx << ' ' << ty << std::endl;
      // assert(0);  bugs
      return getCurPos();
    }

    uint32_t d = static_cast<uint32_t>(direction);

    uint32_t nx = _x + Map::_sdir[d][0];
    uint32_t ny = _y + Map::_sdir[d][1];

    assert(Map::isMoveAble(nx, ny));

    return {nx, ny};
  }

  static std::array<std::pair<uint32_t, uint32_t>, ROBOT_MAX> next_posi;

  bool move(Direction dir) {

    if (_already_moved)
      return false;

    if (dir == Direction::none)
      return false;

    auto nx = _x + Map::_sdir[static_cast<uint32_t>(dir)][0];
    auto ny = _y + Map::_sdir[static_cast<uint32_t>(dir)][1];

    assert(Map::isMoveAble(nx, ny));

    this->_x = nx;
    this->_y = ny;

    robots_actions.push_back(
        {RobotAction::ActionType::move,
         {static_cast<uint8_t>(_id), static_cast<uint8_t>(dir)}});

    _already_moved = true;
    return true;
  }

  bool _already_moved;

  static std::vector<uint32_t> caller_move2;

  bool move2(bool mustgo) {

    if (_already_moved)
      return false;

    Direction dir = Map::getDirection(this->getCurPos(), next_posi[this->_id]);

    uint32_t nx, ny;

    if (dir == Direction::none) {
      nx = this->_x;
      ny = this->_y;
    } else {
      int idx = static_cast<int>(dir);
      nx = this->_x + Map::_sdir[idx][0];
      ny = this->_y + Map::_sdir[idx][1];
    }

    int tid = isThereARobotNow(nx, ny);

    if (tid == -1) { // 无机器人
    _label1:
      return this->move(dir);
    } else if (tid == this->_id) {
      if (not mustgo) {
        goto _label1;
        // return this->move(dir);
      } else {
        goto _label2;
      }
    } else {
    _label2:
      auto p = std::find(std::begin(caller_move2), std::end(caller_move2), tid);

      if (p != std::end(caller_move2)) { // tid 是他的外层,自己就要走
      // _label2:
      _label3:
        std::vector<std::pair<uint32_t, Direction>> there_robots;
        for (int i = 0; i < 4; i++) {
          Direction tdir = static_cast<Direction>(i);
          uint32_t tnx = this->_x + Map::_sdir[i][0];
          uint32_t tny = this->_y + Map::_sdir[i][1];
          if (not Map::isMoveAble(tnx, tny))
            continue;
          int ttid = isThereARobotNow(tnx, tny);
          if (ttid == -1) {
            return this->move(tdir); // 往空地走
          } else {
            there_robots.push_back({ttid, tdir});
          }
        }
        for (auto [rid, rdir] : there_robots) {
          auto p =
              std::find(std::begin(caller_move2), std::end(caller_move2), rid);
          if (p != std::end(caller_move2)) // i 是他的caller
            continue;

          auto &ttrb = robots[rid];
          caller_move2.push_back(rid);
          bool flag = ttrb.move2(true);
          caller_move2.pop_back();
          if (flag) {
            return this->move(rdir);
          }
        }
        // std::abort();
        // assert(false);
        return false; // 死锁了
      } else {        // tid 不是他的外层
        auto &trb = robots[tid];
        caller_move2.push_back(tid);
        bool flag = trb.move2(true);
        caller_move2.pop_back();
        if (flag) {
          return this->move(dir);
        } else {
          goto _label3;
        }
        return flag;
      }
    }

    // Never reach here
    assert(false);
    // std::abort();
    return false;
  }

  bool get() {
    if (_target_cargo_id == static_cast<uint32_t>(-1))
      return false;

    auto &cargo = cargos[_target_cargo_id];

    if (not cargo.isAvailable())
      return false;

    if (not(_x == cargo._origin_x and _y == cargo._origin_y))
      return false;

    assert(_carry_cargo_id == (uint32_t)-1);
    assert(_carryStatus == CarryStatus::empty);

    _carry_cargo_id = _target_cargo_id;
    _target_cargo_id = -1;
    _score = cargo._score;
    _carryStatus = CarryStatus::carrying;

    assert(cargo._taken == false);

    all_values_get += cargos[_carry_cargo_id]._price;

    cargo._taken = true;
    _robot_carry_cnt++;
    robots_actions.push_back(
        {RobotAction::ActionType::get, {static_cast<uint8_t>(_id), 0}});
    return true;
  }
};

// std::array<Robot, ROBOT_MAX> robots;

std::array<Robot, ROBOT_MAX> Robot::robots;
std::array<std::pair<uint32_t, uint32_t>, ROBOT_MAX> Robot::next_posi;
std::vector<uint32_t> Robot::caller_move2;

// 实际上把给货物分配港口,分配机器人的活都干了
class CargoPqItem {
public:
  uint32_t _id;
  // vector<distance,rid> , nearest first
  mutable std::vector<std::pair<uint32_t, uint32_t>> _nearest_robs;

  CargoPqItem() = delete;
  CargoPqItem(uint32_t id) : _id(id) {}
  CargoPqItem(const Cargo &cargo) : _id(cargo._id) {}
  CargoPqItem(const CargoPqItem &item) : _id(item._id) {
    _nearest_robs = item._nearest_robs;
  }
  CargoPqItem(CargoPqItem &&item) {
    _id = item._id;
    _nearest_robs = std::move(item._nearest_robs);
  }
  CargoPqItem &operator=(const CargoPqItem &item) {
    _id = item._id;
    _nearest_robs = item._nearest_robs;
    return *this;
  }

  // return the distance to robot, if robot with cargo ,return real distance ;
  // else return manhattan distance
  uint32_t distanceToRobot(uint32_t rid) const {
    const auto &cargo = cargos[_id];
    const auto &robo = Robot::robots[rid];
    if (unlikely(not Map::isConnected({cargo._origin_x, cargo._origin_y},
                                      {robo._x, robo._y})))
      return -1;
    if (robo.isWithCargo()) {
      const auto &withcargo = cargos[robo._carry_cargo_id];
      const auto robo_to_berth =
          Map::getDistanceToBerth(robo._x, robo._y, withcargo._berth_id);
      const auto berth_to_cargo = Map::getDistanceToBerth(
          cargo._origin_x, cargo._origin_y, withcargo._berth_id);
      return robo_to_berth + berth_to_cargo;
    } else {
      int rd2b = Map::getDistanceToBerth(robo._x, robo._y, cargo._berth_id);
      int cd2b = Map::getDistanceToBerth(cargo._origin_x, cargo._origin_y,
                                         cargo._berth_id);

      return Map::manhattanDistance({cargo._origin_x, cargo._origin_y},
                                    {robo._x, robo._y}) +
             abs(rd2b - cd2b);
    }
  }

  // vector<distance,rid> , nearest first
  std::vector<std::pair<uint32_t, uint32_t>>
  distToRobots(bool sorted = true) const {
    std::vector<std::pair<uint32_t, uint32_t>> ret;
    for (auto &&robot : Robot::robots) {
      if (unlikely(robot._status == Robot::Status::useless))
        continue;
      int dis = this->distanceToRobot(robot._id);
      if (dis == -1)
        continue;
      ret.push_back({dis, robot._id});
    }
    if (sorted)
      std::sort(begin(ret), end(ret));
    return ret;
  }

  // {distance , robot id}
  std::pair<uint32_t, uint32_t> nearestRobot() const {
    const auto &diss = this->distToRobots(false);
    if (unlikely(diss.empty()))
      return {-1, -1};
    const auto p = std::min_element(begin(diss), end(diss));
    return *p;
  }

  float avgDistToRobots(uint32_t iter_max = 3) const {
    this->_nearest_robs = this->distToRobots();
    const auto &rs = this->_nearest_robs;

    uint32_t cnt = 0;
    uint32_t sum = 0;

    for (auto [dis, rid] : rs) {
      if (cnt >= iter_max)
        break;
      if (dis == static_cast<uint32_t>(-1))
        break;
      sum += dis;
      ++cnt;
    }

    if (unlikely(cnt == 0))
      return std::numeric_limits<float>::max();
    return static_cast<float>(sum) / cnt;
  }

  float getScore() const {
    auto &cargo = cargos[_id];
    if (cargo._taken)
      return std::numeric_limits<float>::min();

    int32_t remain_frame = cargo._disappear_frame - getCurrentFrame();
    if (remain_frame <= 0)
      return std::numeric_limits<float>::min();

    const auto nb_id = cargo._berth_id;
    const auto nb_dis =
        Map::getDistanceToBerth(cargo._origin_x, cargo._origin_y, nb_id);
    const auto nb_time = berths[nb_id]._time;

    const float avg_rob = this->avgDistToRobots(ROBOT_MAX / 5);
    remain_frame -= (nb_dis + avg_rob); // ! 最好用最近的机器人的距离
    if (remain_frame < 0)
      return std::numeric_limits<float>::min();

    const float price = cargo._price;
    float sc;
    // const float sc = price / (nb_dis + avg_rob + nb_time);
    if (getCurrentFrame() + FRAME_LAST >= FRAME_MAX) {
      sc = (std::exp(price / 200)) / (avg_rob);
    } else {
      sc = (std::exp(price / 200)) / (nb_dis * 0.5 + avg_rob * 1.5);
    }
    cargo._score = sc;
    return sc;
  }

  bool operator<(const CargoPqItem &rhs) const {
    assert(_id < cargos.size());
    assert(rhs._id < cargos.size());
    return this->getScore() < rhs.getScore();
  }

  bool operator==(const CargoPqItem &rhs) const { return _id == rhs._id; }
  bool operator>(const CargoPqItem &rhs) const {
    if (unlikely(*this == rhs))
      return false;
    return rhs < *this;
  }
};

// 所有在地图上还没有被拿的所有货物,可能包含过期的货物
std::priority_queue<CargoPqItem> cargos_pq;
// std::vector<CargoPqItem> cargo_pqs;

void berth_init() {
  auto p = std::min_element(
      std::begin(berths), std::end(berths),
      [](const Berth &lhs, const Berth &rhs) { return lhs._time < rhs._time; });

  auto min_time = p->_time;
  for (auto &berth : berths) {
    if (min_time + FRAME_SHIP_SWITH_FROM_BERTH < berth._time) {
      berth_govp_via[berth._id] = p->_id;
      std::cerr << "berth_init: "
                << " " << min_time << " " << p->_id << " " << berth._id << " "
                << berth._time << std::endl;
    } else {
      berth_govp_via[berth._id] = berth._id;
    }
  }
}

void initialization() {
  cargos.reserve(10 * FRAME_MAX);
  robots_actions.reserve(ROBOT_MAX);
  ships_actions.reserve(SHIP_MAX);

  Map::readmap();

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
    // berths[id]._cargo_ongoing = 0;

    Map::processConnectedBerth(id);
  }

  berth_init();

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

  auto _idx = cargos.size();

  uint32_t K;
  // 场上新增货物的数量 K
  scanf("%u", &K);
  for (uint32_t i = 0; i < K; ++i) {
    uint32_t x, y, price;
    // 货物的位置坐标、金额
    scanf("%u %u %u", &x, &y, &price);
    if (unlikely(not Map::isConnectedToAnyBerth(x, y))) // 货物在封闭区域,忽略之
      continue;

    cargos.push_back({static_cast<uint32_t>(cargos.size()), x, y, price,
                      frame_id + FRAME_CARGO_REMAIN, false,
                      static_cast<uint32_t>(0), 0.0});

    cargos.back()._berth_id = Map::nearestBerth(x, y).second;

    // cargos_pq.push(cargos.back());
  }

  for (uint8_t id = 0; id < ROBOT_MAX; ++id) {
    uint32_t carryflag, x, y, status;
    // carryflag: 0 表示未携带物品,1 表示携带物品。
    // x,y 机器人所在位置坐标
    // status:  0 表示恢复状态,1 表示正常运行状态
    scanf("%u %u %u %u", &carryflag, &x, &y, &status);
    Robot::robots[id]._id = id;
    Robot::robots[id]._x = x;
    Robot::Robot::robots[id]._y = y;
    Robot::robots[id]._already_moved = false;

    if (unlikely(getCurrentFrame() < 2)) {
      Robot::robots[id]._score = 0;
      Robot::robots[id]._carryStatus = Robot::CarryStatus::empty;
      Robot::robots[id]._target_cargo_id = -1;
      Robot::robots[id]._carry_cargo_id = -1;
    }

    if (unlikely(not Map::isConnectedToAnyBerth(x, y)))
      Robot::robots[id]._status = Robot::Status::useless;
    else
      Robot::robots[id]._status = static_cast<Robot::Status>(status);

    if (carryflag == 0) {

      assert(Robot::robots[id]._carryStatus == Robot::CarryStatus::empty);

      Robot::robots[id]._carryStatus = Robot::CarryStatus::empty;
      Robot::robots[id]._carry_cargo_id = -1;

    } else if (carryflag == 1) {
      // if (Robot::robots[id]._carry_cargo_id == -1) {
      //   Robot::robots[id]._carryStatus = Robot::CarryStatus::empty;
      // }
      // 本地状态还没拿到货, 评测机认为拿到货了
      assert(Robot::robots[id]._carryStatus == Robot::CarryStatus::carrying);
      assert(Robot::robots[id]._carry_cargo_id != (uint32_t)-1); // ! bug here

      Robot::robots[id]._carryStatus = Robot::CarryStatus::carrying;
    }
  }

  while (_idx < cargos.size()) {
    const auto &cargo = cargos[_idx++];
    cargos_pq.push(cargo);
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

    if (status == 2) {
      std::cerr << "ship " << id << " is waiting at berth " << berth_id
                << std::endl;
    }

    if (status == 1 and berth_id != -1) {
      const auto &bert = berths[berth_id];
      if (not bert._ships_via.empty()) {
        auto [frame, shid, tid] = berths[berth_id]._ships_via.front();
        if (shid == id) {
          std::cerr << id << ' ';
          std::cerr << bert._ships_here.front().first << ' '
                    << bert._ships_here.front().second << std::endl;
          assert(bert.countShipsNow() == 0);
        } else {
          assert(berths[berth_id].curShip().second == id);
        }
      }
      if (bert.countShipsNow()) {
        if (berths[berth_id].curShip().second != id) {
          std::cerr << berth_id << ' ' << berths[berth_id].curShip().second
                    << ' ' << id << std::endl;
          auto [frame, shid, tid] = berths[berth_id]._ships_via.front();
          std::cerr << frame_current << ' ' << frame << ' ' << shid << ' '
                    << tid << std::endl;
        }
        assert(berths[berth_id].curShip().second == id);
      }
    }

    if (status == 1 and berth_id == -1) {
      ships[id].clear();
    }
  }

  char okk[32];
  scanf("%s", okk);
  return frame_id;
};

class BerthForVirtualPoint {
public:
  uint32_t _bid;
  BerthForVirtualPoint() = delete;
  BerthForVirtualPoint(uint32_t bid) : _bid(bid) {}
  BerthForVirtualPoint(const Berth &b) : _bid(b._id) {}

  float getScore() const {
    const auto &lhs = berths[_bid];
    if (lhs.countShips())
      return std::numeric_limits<float>::min(); // min float

    const float lhs_value = lhs.sumCargosValueNow();
    const float lhs_cnt = lhs.countCargosNow();
    const auto lhs_speed = lhs._velocity;
    const auto lhs_time = lhs._time;

    return lhs_value / (lhs_cnt / lhs_speed + lhs_time);
  }

  bool operator<(BerthForVirtualPoint r) const {
    return this->getScore() < r.getScore();
  }
};

class BerthForShipAtBerth {
public:
  uint32_t _bid, _shipid;
  BerthForShipAtBerth() = delete;
  BerthForShipAtBerth(uint32_t bid, uint32_t sid) : _bid(bid), _shipid(sid) {}

  float getScore() const {
    const auto &ship = ships[_shipid];
    if (_bid == static_cast<uint32_t>(-1)) {
      const float cur_value = ship._values;
      const auto &bert = berths[ship._berth_id];
      auto viabid = berth_govp_via[ship._berth_id];
      auto bert_time = bert._time;
      if (viabid != ship._berth_id) {
        bert_time = FRAME_SHIP_SWITH_FROM_BERTH + berths[viabid]._time;
      }
      return cur_value / bert_time;
      // return 0;
    }

    const auto &bert = berths[_bid];
    if (bert.countShips())
      return std::numeric_limits<float>::min();

    const float cur_value = ship._values;
    const auto remain_size = Ship::capacity - ship._size;
    const auto bert_cnt = bert.countCargosNow();

    const auto can_get = std::min(remain_size, bert_cnt);
    uint32_t extra_value = 0;
    for (uint32_t i = 0; i < can_get; ++i) {
      const auto [price, cid] = bert._cargos_here[i];
      extra_value += price;
    }

    const auto bert_time = bert._time;
    const float bert_speed = bert._velocity;

    return (cur_value + extra_value) /
           (FRAME_SHIP_SWITH_FROM_BERTH + bert_time + (can_get / bert_speed));
  }

  bool operator<(BerthForShipAtBerth r) const {
    return this->getScore() < r.getScore();
  }
};

void shipsUpdate() {

  static std::vector<uint32_t> ships_at_vp;
  ships_at_vp.reserve(BERTH_MAX);
  static std::vector<uint32_t> ships_at_berth;
  ships_at_berth.reserve(BERTH_MAX);

  ships_at_vp.clear();
  ships_at_berth.clear();

  for (const auto &sh : ships) {
    if (sh._status != Ship::Status::normal)
      continue;
    if (sh._berth_id == -1) // 在虚拟点
      ships_at_vp.push_back(sh._id);
    else {
      const auto &bert = berths[sh._berth_id];
      if (not bert._ships_via.empty()) {
        auto [frame, shid, tbid] = bert._ships_via.front();
        std::cerr << "ships_at_berth : " << frame_current << ' ' << frame << ' '
                  << bert._id << ' ' << shid << ' ' << tbid << ' ' << sh._id
                  << std::endl;
        if (sh._id == shid)
          continue;
      }
      ships_at_berth.push_back(sh._id);
    }
  }

  for (auto &bert : berths) {
    while (not bert._ships_via.empty()) {
      auto [frame, shid, tbid] = bert._ships_via.front();
      std::cerr << "shipsUpdate : " << frame_current << ' ' << frame << ' '
                << bert._id << ' ' << shid << ' ' << tbid << std::endl;
      if (getCurrentFrame() >= frame) {
        bert._ships_via.pop_front();
        if (tbid == -1) {
          ships[shid].go();
        } else {
          ships[shid].ship(tbid);
          // auto &bert = berths[tbid];
          // bert.loadShip(
          //     {getCurrentFrame() + FRAME_SHIP_SWITH_FROM_BERTH, shid});
        }
      } else {
        break;
      }
    }
  }

  static std::vector<BerthForVirtualPoint> vp_candidates;
  vp_candidates.reserve(BERTH_MAX);
  vp_candidates.clear();

  if (not ships_at_vp.empty()) {

    {
      for (const auto &bert : berths) {
        if (bert.countShips())
          continue;
        vp_candidates.push_back(bert._id);
      }
      std::sort(begin(vp_candidates), end(vp_candidates));
    }

    {
      for (const auto shid : ships_at_vp) {
        auto &ship = ships[shid];
        if (unlikely(vp_candidates.empty()))
          break;
        const auto target_bid = vp_candidates.back()._bid;
        const float score = vp_candidates.back().getScore();
        vp_candidates.pop_back();
        if (score <= 0) // Magic Number
          break;        // 船不动

        auto &bert = berths[target_bid];
        if (berth_govp_via[target_bid] == target_bid) {
          ship.ship(target_bid);
          bert.loadShip({getCurrentFrame() + bert._time, ship._id});
        } else {
          // 绕路
          auto viabid = berth_govp_via[target_bid];
          ship.ship(viabid);
          auto time = berths[viabid]._time;
          berths[viabid]._ships_via.push_back(
              {getCurrentFrame() + time, shid, target_bid});
          bert.loadShip(
              {getCurrentFrame() + time + FRAME_SHIP_SWITH_FROM_BERTH, shid});
        }
      }
    }
  }

  static std::vector<BerthForShipAtBerth> berth_candidates;
  berth_candidates.reserve(BERTH_MAX);
  berth_candidates.clear();

  if (likely(not ships_at_berth.empty())) {

    const uint32_t free_upbound = getRandom(50, 100);

    for (const auto shid : ships_at_berth) {
      auto &ship = ships[shid];
      auto &berth = berths[ship._berth_id];
      auto cnt = berth.cargoGoOnBoardInOneFrame();

      auto govptime = goVpNeedTime(berth._id);

      if (ship._size == Ship::capacity or
          getCurrentFrame() + govptime + 1 >= FRAME_MAX) {
        berth.leaveShip();

        auto nowbid = ship._berth_id;
        if (berth_govp_via[nowbid] == nowbid) {
          ship.go();
        } else {
          //
          auto viabid = berth_govp_via[nowbid];
          ship.ship(viabid);
          auto &viab = berths[viabid];
          viab._ships_via.push_back(
              {getCurrentFrame() + FRAME_SHIP_SWITH_FROM_BERTH, shid, -1});
        }

        continue;
      }

      if (cnt > 0) {
        if (getCurrentFrame() + FRAME_LAST >= FRAME_MAX) {
          ship._free_frame = 0;
        }
        continue;
      }

      ship._free_frame++;
      if (ship._free_frame > free_upbound) {

        {
          for (const auto &bert : berths) {
            if (bert.countShips())
              continue;
            berth_candidates.push_back({bert._id, shid});
          }
          berth_candidates.push_back({static_cast<uint32_t>(-1), shid});
        }

        auto maxp =
            std::max_element(begin(berth_candidates), end(berth_candidates));

        int target = maxp->_bid;

        if (getCurrentFrame() + FRAME_LAST >= FRAME_MAX) {
          if (target == -1) {
            berth_candidates.pop_back();
            maxp = std::max_element(begin(berth_candidates),
                                    end(berth_candidates));
            target = maxp->_bid;
          }
        }

        if (target == -1) { // go to virtual point
          auto nowbid = ship._berth_id;
          auto viabid = berth_govp_via[nowbid];
          if (viabid == nowbid) {
            ship.go();
          } else {
            ship.ship(viabid);
            auto &viab = berths[viabid];
            viab._ships_via.push_back(
                {getCurrentFrame() + FRAME_SHIP_SWITH_FROM_BERTH, shid, -1});
          }
          berth.leaveShip();
        } else {
          auto &target_bert = berths[target];
          auto govptime1 = goVpNeedTime(target);
          if (getCurrentFrame() + govptime1 + FRAME_SHIP_SWITH_FROM_BERTH >=
              FRAME_MAX) {
            continue;
          }
          ship.ship(target);
          target_bert.loadShip(
              {getCurrentFrame() + FRAME_SHIP_SWITH_FROM_BERTH, ship._id});
          berth.leaveShip();
        }
      }
    }
  }
}

void robotsUpdate() {

  static std::unordered_set<uint32_t> robs_avaiable;
  robs_avaiable.clear();

  for (const auto &rob : Robot::robots) {
    if (rob._status == Robot::Status::recovering)
      continue;
    if (rob._target_cargo_id != static_cast<uint32_t>(-1)) // 已经被分配了
      continue;

    robs_avaiable.insert(rob._id);
  }

  static std::vector<CargoPqItem> _tmp;
  _tmp.reserve(FRAME_MAX);

  // std::cerr << cargos_pq.size() << " " << robs_avaiable.size() <<
  // std::endl;

  while ((not cargos_pq.empty()) and robs_avaiable.size()) {
    auto citem = cargos_pq.top();
    cargos_pq.pop();

    const auto &c = cargos[citem._id];

    if (not c.isAvailable())
      continue;

    bool dispatched = false;
    const auto &nrs = citem._nearest_robs;

    // {
    //   for (auto &&item : nrs) {
    //     // dis ,rid
    //     uint32_t cx = c._origin_x;
    //     uint32_t cy = c._origin_y;

    //     // item.first +=
    //   }
    // }

    constexpr unsigned long bestof = ROBOT_MAX / 5;
    const uint32_t cnt = std::min(bestof, nrs.size());
    for (uint32_t i = 0; i < cnt; i++) {
      const auto [dis, rid] = nrs[i];
      if (robs_avaiable.count(rid)) { // 可以分配
        auto &rob = Robot::robots[rid];
        dispatched = true;
        robs_avaiable.erase(rid);
        rob._target_cargo_id = c._id;
        if (rob._carryStatus == Robot::CarryStatus::empty)
          rob._score = c._score;
        break;
      }
    }
    if (not dispatched) {
      _tmp.push_back(std::move(citem));
    }
  }

  for (auto &&cargo : _tmp) {
    const auto &c = cargos[cargo._id];
    if (not c.isAvailable())
      continue;
    cargos_pq.push(std::move(cargo));
  }
  _tmp.clear();

  static std::vector<std::pair<float, uint32_t>> robots_priority;
  robots_priority.reserve(ROBOT_MAX);
  robots_priority.clear();

  for (auto &&rob : Robot::robots) {
    float __sc = rob._score;
    if (rob._carryStatus == Robot::CarryStatus::carrying) { // 拿货的优先级高
      if (getCurrentFrame() + FRAME_LAST >= FRAME_MAX) {
        __sc += 1000000;
      }
    }

    robots_priority.push_back({__sc, rob._id});
  }
  std::sort(std::rbegin(robots_priority), std::rend(robots_priority));

  static bool robs_do_action_flags[ROBOT_MAX];
  memset(robs_do_action_flags, 0, sizeof(robs_do_action_flags));

  {
    for (auto [sc, rid] : robots_priority) {
      auto &rob = Robot::robots[rid];
      if (rob._target_cargo_id == static_cast<uint32_t>(-1))
        continue;
      auto &c = cargos[rob._target_cargo_id];

      auto mdis =
          Map::manhattanDistance({rob._x, rob._y}, {c._origin_x, c._origin_y});

      int remain = c._disappear_frame - getCurrentFrame();

      if (remain <= mdis) {
        rob._target_cargo_id = -1;
      }
    }
  }

  {
    // 装货物，卸货
    for (auto [sc, rid] : robots_priority) {
      auto &rob = Robot::robots[rid];
      if (rob.isInBerth()) {
        // robs_do_action_flags[rid] = rob.pull();
        rob.pull();
      } else if (rob.canGetCargo()) {
        // robs_do_action_flags[rid] = rob.get();
        rob.get();
      }
    }
  }

  for (const auto &r : Robot::robots) {
    Robot::next_posi[r._id] = r.getNextPos();
  }

  {
    for (auto [sc, rid] : robots_priority) {
      // if (robs_do_action_flags[rid])
      // continue;
      auto &&rob = Robot::robots[rid];

      if (rob._status == Robot::Status::useless)
        continue;

      // rob.move();
      Robot::caller_move2.push_back(rid);
      rob.move2(false);
      Robot::caller_move2.pop_back();
    }
    // unsetAllBarriers();
  }
}

void frameUpdate() {
  // TimeCounter _tc{__FUNCTION__};

  robotsUpdate();

  shipsUpdate();
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
    frameInput();
    frameUpdate();
    frameOutput();
    frameDone();
  }
  std::cerr << all_values_get << ' ' << all_values_pull << std::endl;
  return 0;
}