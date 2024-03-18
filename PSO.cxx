#include <iostream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <array>
#include <utility>
#include <random>

constexpr unsigned int ROBOT_NUM = 10;
constexpr double conflict_punish = -1000.0;

enum class Direction : uint8_t{
    right = 0,
    up = 1,
    left = 2,
    down = 3,
    none = 4,
};

class Particle{
public:
    std::array<int32_t , ROBOT_NUM> cargo_val;
    std::array<std::pair<int8_t , int8_t>, ROBOT_NUM> sourcePoint;   //对应每个机器人的出发点与目标点
    std::array<std::pair<int8_t, int8_t>, ROBOT_NUM> targetPoint;
    std::array<std::pair<int8_t, int8_t>, ROBOT_NUM> nextPoint;
    std::array<Direction, ROBOT_NUM> regular_position;   //输出的规范当前帧的所有机器人的一步策略

    //初始化v0 = 0
    void Initialize(){
        _velo.fill(0.0);
        for (unsigned int i = 0;i < ROBOT_NUM; ++i){
            _xposi[i] = static_cast<double>(rand() % 5);   //初始位置为随机
        }
    }

    //根据粒子当前状态更新机器人在地图的位置
    void update_ROBloc(){
        regular();
        for(uint8_t i = 0; i < ROBOT_NUM; ++i){
            switch(regular_position[i]){
                case Direction::right:
                    nextPoint[i].first = sourcePoint[i].first;
                    nextPoint[i].second = sourcePoint[i].second+1;
                    break;
                case Direction::up:
                    nextPoint[i].first = sourcePoint[i].first-1;
                    nextPoint[i].second = sourcePoint[i].second;
                    break;
                case Direction::left:
                    nextPoint[i].first = sourcePoint[i].first;
                    nextPoint[i].second = sourcePoint[i].second-1;
                    break;
                case Direction::down:
                    nextPoint[i].first = sourcePoint[i].first+1;
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

    //输出可以为负数        ！！！！没写冲突的惩罚量
    double frame_benefit(){
        double benefit = 0;
        update_ROBloc();
        if(!conflict_check()){
            for(uint8_t i = 0; i < ROBOT_NUM; ++i){
                if(regular_position[i] == Direction::none) continue;  //当这个机器人在原处等待,跳过这个机器人计算收益
                benefit += static_cast<double>(cargo_val[i]/(calculate_dis(sourcePoint[i], targetPoint[i]) -
                                                             calculate_dis(nextPoint[i], targetPoint[i])));
            }
        }
        else benefit = conflict_punish;
        return benefit;
    }

    //检查是否需要更新pbest
    void check_update(){
        double current_benefit = frame_benefit();
        if(current_benefit > Pbest_benefit){
            Pbest_benefit = current_benefit;
            Pbest_position = regular_position;
        }
    }

    //更新velocity调整策略参数    ---其中 w~(0, 1) c_1, c_2 一般都设置为 2
    void update_velo(double w, double c_1, double c_2, std::array<Direction, ROBOT_NUM> Gbest_position){
        std::random_device rd;
        std::mt19937  gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        double r_1 = dis(gen);    //生成 r_1,r_2 0~1之间的数
        double r_2 = dis(gen);

        for(uint8_t i = 0; i < ROBOT_NUM; ++i){
            _velo[i] = w*_velo[i] + c_1*r_1*(static_cast<double>(Pbest_position[i])-_xposi[i]) +
                    c_2*r_2*(static_cast<double>(Gbest_position[i])-_xposi[i]);
        }
    }

    //更新_xposition策略参数    必须在更新velocity参数更新之后才更新
    void update_xposition(){
        for(uint8_t i = 0; i < ROBOT_NUM; ++i){
            _xposi[i] = _velo[i] + _xposi[i];
        }
    }

    //固定更新顺序
    void update_para(double w, double c_1, double c_2, std::array<Direction, ROBOT_NUM> Gbest_position){
        update_velo(w, c_1, c_2, Gbest_position);
        update_xposition();
    }

    //当前策略是否会产生冲突   (但是这种比较方式引入了不必要的比较，分开比较写着麻烦)
    bool conflict_check(){
        for(uint8_t i = 0; i < ROBOT_NUM; ++i){
            for(uint8_t j = 0; j < ROBOT_NUM, j != i; ++j){
                if(nextPoint[i] == sourcePoint[j] || nextPoint[i] == sourcePoint[j])
                return false;
            }
        }
        return true;
    }

    //两点之间曼哈顿距离
    int32_t calculate_dis(std::pair<int, int> source_pt, std::pair<int, int> target_pt){
        return abs(source_pt.first - target_pt.first) + abs(source_pt.second - target_pt.second);
    }

    //规范输出
    void regular(){
        for(unsigned int i = 0; i < ROBOT_NUM; i++){
            regular_position[i] = static_cast<Direction>(_xposi[i]);     //应该根据阈值规范输出，不应该是简单的类型转换（稍后修改）
        }
    }


private:
    std::array<double, ROBOT_NUM> _velo;   //定义doulble类型在更新参数时进行运算
    std::array<double, ROBOT_NUM> _xposi;
    int32_t Pbest_benefit;
    std::array<Direction, ROBOT_NUM> Pbest_position;

};

class PSO_Algorithm : public Particle{
public:
    //传入初始参数
    PSO_Algorithm(uint8_t particle_num, double w, double c_1, double c_2){
        this->w = w;
        this->c_1 = c_1;
        this->c_2 = c_2;
        this->particle_num = particle_num;
    }

    void Initialize(){
        std::vector<Particle> Particles;
        Particles.resize(particle_num);
        for(auto &particle : Particles){
            particle.Initialize();    //为每个粒子进行初始化
        }
        Gbest_benefit = Particles[0].frame_benefit();
//        Gbest_position = Particles[0].
    }

    //更新gpest   并用static变量记录gbest未更新的次数

private:
    double w;  //惯性参数
    double c_1, c_2; //学习因子
    uint8_t particle_num;
    int32_t Gbest_benefit;
    std::array<Direction, ROBOT_NUM> Gbest_position;
};


