#pragma once

#include <iostream>
#include <vector>
// #include <cstdio>
#include <array>
#include <cstdlib>
#include <random>
#include <utility>
// #include <limits>
#include <algorithm>

constexpr unsigned int ROBOT_NUM = 10;
// constexpr double Neg_Infinity = std::numeric_limits<double>::lowest();
// //负无穷
constexpr double conflict_punish = -1000.0;

// Order is IMPORTANT!
enum class Direction : uint8_t {
  right = 0,
  left = 1,
  up = 2,
  down = 3,
  none,
};

static std::array<std::pair<int, int>, ROBOT_NUM> src_pt;
static std::array<std::pair<int, int>, ROBOT_NUM> targ_pt;
static std::array<int32_t, ROBOT_NUM> cg_val;

class Particle {
public:
  // 初始化v0 = 0
  void Initialize() {
    _velo.fill(0.0);
    Pbest_benefit = 0;
    for (unsigned int i = 0; i < ROBOT_NUM; ++i) {
      _xposi[i] = static_cast<double>(rand() % 5); // 初始策略为随机
      Pbest_position[i] = Direction::none; // 初始的最好策略为不动
    }

    regular();
  }

  // 传入sourcepoint,targetpoint, cargo_val
  void set(std::array<std::pair<int, int>, ROBOT_NUM> src_pt,
           std::array<std::pair<int, int>, ROBOT_NUM> targ_pt,
           std::array<int32_t, ROBOT_NUM> cg_val) {
    this->sourcePoint = std::move(src_pt);
    this->targetPoint = std::move(targ_pt);
    this->cargo_val = std::move(cg_val);
  }

  // 检查是否需要更新pbest
  void check_update() {
    double current_benefit = frame_benefit();
    if (current_benefit > Pbest_benefit) {
      Pbest_benefit = current_benefit;
      Pbest_position = regular_position;
    }
  }

  // 固定更新顺序     （先更新Gbest参数）
  void update_para(double w, double c_1, double c_2,
                   std::array<Direction, ROBOT_NUM> Gbest_position) {
    update_velo(w, c_1, c_2, Gbest_position);
    update_xposition();
  }

  // 友元声明
  friend class PSO_Algorithm;

private:
  std::array<int32_t, ROBOT_NUM> cargo_val;
  std::array<std::pair<int, int>, ROBOT_NUM>
      sourcePoint; // 对应每个机器人的出发点与目标点
  std::array<std::pair<int, int>, ROBOT_NUM> targetPoint;
  std::array<std::pair<int, int>, ROBOT_NUM> nextPoint;
  std::array<Direction, ROBOT_NUM>
      regular_position; // 输出的规范当前帧的所有机器人的一步策略
  std::array<double, ROBOT_NUM> _velo; // 定义doulble类型在更新参数时进行运算
  std::array<double, ROBOT_NUM> _xposi;
  int32_t Pbest_benefit;
  std::array<Direction, ROBOT_NUM> Pbest_position;

  // 规范输出    考虑越界问题，并且考虑超阈值处理
  void regular() {
    for (unsigned int i = 0; i < ROBOT_NUM; i++) {
      regular_position[i] = static_cast<Direction>(
          _xposi
              [i]); // 应该根据阈值规范输出，不应该是简单的类型转换（稍后修改）
    }
  }

  // 根据粒子当前状态更新机器人在地图的位置
  void update_ROBloc() {
    regular();
    for (uint i = 0; i < ROBOT_NUM; ++i) {
      switch (regular_position[i]) {
      case Direction::right:
        nextPoint[i].first = sourcePoint[i].first;
        nextPoint[i].second = sourcePoint[i].second + 1;
        break;
      case Direction::up:
        nextPoint[i].first = sourcePoint[i].first - 1;
        nextPoint[i].second = sourcePoint[i].second;
        break;
      case Direction::left:
        nextPoint[i].first = sourcePoint[i].first;
        nextPoint[i].second = sourcePoint[i].second - 1;
        break;
      case Direction::down:
        nextPoint[i].first = sourcePoint[i].first + 1;
        nextPoint[i].second = sourcePoint[i].second;
        break;
      case Direction::none:
        nextPoint[i].first = sourcePoint[i].first;
        nextPoint[i].second = sourcePoint[i].second;
        break;
      default:
        break;
      }
    }
  }

  // 更新velocity调整策略参数    ---其中 w~(0, 1) c_1, c_2 一般都设置为 2
  void update_velo(double w, double c_1, double c_2,
                   std::array<Direction, ROBOT_NUM> Gbest_position) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);
    double r_1 = dis(gen); // 生成 r_1,r_2 0~1之间的数
    double r_2 = dis(gen);

    for (uint i = 0; i < ROBOT_NUM; ++i) {
      _velo[i] =
          w * _velo[i] +
          c_1 * r_1 * (static_cast<double>(Pbest_position[i]) - _xposi[i]) +
          c_2 * r_2 * (static_cast<double>(Gbest_position[i]) - _xposi[i]);
    }
  }

  // 更新_xposition策略参数    必须在更新velocity参数更新之后才更新
  void update_xposition() {
    for (uint i = 0; i < ROBOT_NUM; ++i) {
      _xposi[i] = _velo[i] + _xposi[i];
    }
  }

  // 当前策略是否会产生冲突
  // (但是这种比较方式引入了不必要的比较，分开比较写着麻烦)
  // //还有不能超过地图边界
  bool conflict_check() {
    for (uint i = 0; i < ROBOT_NUM; ++i) {
      for (uint j = 0; j < ROBOT_NUM and j != i; ++j) {
        if (nextPoint[i] == sourcePoint[j] || nextPoint[i] == nextPoint[j])
          return false;
      }
    }
    return true;
  }

  // 输出可以为负数        ！！！！没写冲突的惩罚量
  double frame_benefit() {
    double benefit = 0;
    update_ROBloc();
    // 不存在冲突
    if (!conflict_check()) {
      for (uint i = 0; i < ROBOT_NUM; ++i) {
        if (regular_position[i] == Direction::none)
          continue; // 当这个机器人在原处等待,跳过这个机器人计算收益
        benefit += static_cast<double>(
            cargo_val[i] / (calculate_dis(sourcePoint[i], targetPoint[i]) -
                            calculate_dis(nextPoint[i], targetPoint[i])));
      }
    } else
      benefit = conflict_punish;
    return benefit;
  }

  // 两点之间曼哈顿距离
  int32_t calculate_dis(std::pair<int, int> source_pt,
                        std::pair<int, int> target_pt) {
    return abs(source_pt.first - target_pt.first) +
           abs(source_pt.second - target_pt.second);
  }
};

class PSO_Algorithm {
public:
  // 传入初始参数
  PSO_Algorithm(uint particle_num, uint iteration, double w, double c_1,
                double c_2) {
    this->w = w;
    this->c_1 = c_1;
    this->c_2 = c_2;
    this->particle_num = particle_num;
    this->iteration = iteration;
  }

  std::vector<Particle> Particles;

  void Initialize() {
    Particles.resize(particle_num);
    for (auto &particle : Particles) {
      particle.set(src_pt, targ_pt, cg_val); // 设置目标点与出发点并给出货物价值
      particle.Initialize();                 // 为每个粒子进行初始化
      particle.check_update(); // 更新初始的随机策略的参数
                               // （如果随机策略为负数的更新！！需要考虑！！！）
    }
    // 降序排序
    std::sort(Particles.begin(), Particles.end(),
              [](const Particle &a, const Particle &b) {
                return a.Pbest_benefit > b.Pbest_benefit;
              });

    // 初始化Gbest参数为初始Pbest中的最优（更新了一次）
    Gbest_benefit = Particles.begin()->Pbest_benefit;
    Gbest_position = Particles.begin()->Pbest_position;
  }

  // 进行迭代直到达到收敛条件或者已经达到迭代次数
  void run() {
    bool update_flag; // update_flag用于检测当前轮次Gbest是否更新
    static uint noneupdate_num = 0;
    for (uint i = 0; i < iteration; ++i) {
      for (auto &particle : Particles) {
        // 先更新上次结果的几个参数，然后再开始本轮迭代
        particle.update_para(w, c_1, c_2,
                             Gbest_position); // 更新位置_xposi,与速度参数_velo
        particle.check_update();              // 更新Pbest
      }
      // 粒子Pbest排序
      std::sort(Particles.begin(), Particles.end(),
                [](const Particle &a, const Particle &b) {
                  return a.Pbest_benefit > b.Pbest_benefit;
                });
      // 更新Gbest参数
      if (Particles.begin()->Pbest_benefit > Gbest_benefit) {
        Gbest_benefit = Particles.begin()->Pbest_benefit;
        Gbest_position = Particles.begin()->Pbest_position;
        update_flag = true; // 标志Gbest更新
        noneupdate_num = 0;
      } else {
        update_flag = false;
        if (++noneupdate_num > NoneUpdate_threshold)
          break; // 满足收敛条件
      }
    }
  }

private:
  double w;          // 惯性参数
  double c_1, c_2;   // 学习因子
  uint particle_num; // 粒子数量
  uint iteration;    // 迭代次数
  int32_t Gbest_benefit;
  std::array<Direction, ROBOT_NUM> Gbest_position;
  const uint NoneUpdate_threshold = 5;
};
