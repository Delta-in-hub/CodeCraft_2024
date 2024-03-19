# 2024华为软件精英挑战赛
> https://bbs.huaweicloud.com/forum/forum-0168144383617537003-1.html
> https://competition.huaweicloud.com/advance/1000042021/introduction



需要修改以下路径

```cpp
  {
    auto p = freopen("~/workspace/huawei2024/maps/map1.txt", "r", stdin);
    assert(p);
  }
```


[A-Star（A*）寻路算法原理与实现](https://zhuanlan.zhihu.com/p/385733813)

若图形中只允许朝上下左右四个方向移动：使用曼哈顿距离


- 本地,评测机可能有两种状态,如何更新状态?
  - Map 仅维护单纯的地图，不维护机器人和货物的状态
  - 对地图进行预处理
- 机器人选择去接哪个货物?
  - 按距离?按价格?
  - 两个机器人抢一个货物,如何解决? (如此没抢到的机器人做了无用功)
- 机器人接到货物后,如何选择港口?
  - 按距离?
  - 一个码头可以无限堆积货物,FIIO
- 轮船如何选择码头?
  - 轮船可以到码头排队,FIFO
  - 轮船可以在码头间切换
  - 轮船没装满就发船吗?
- 考虑极端场景
  - 不连通


流程：

- [ ] 给货物分配机器人
  - [ ] 货物价值排序需要考虑： 自身价值，能到达的码头个数，码头装卸货物的速度，与码头的距离，与机器人的距离，货物剩余的帧数
- [ ] 机器人寻路，捡起货物，向码头移动
- [ ] 机器人到码头卸货
- [ ] 轮船到码头
- [ ] 轮船装货


BUGS:
- map6有bug,应该是死循环
- 机器人防碰撞有问题
- 轮船应该考虑转泊位
- 货物只考虑最近的泊位放不好,也应该考虑轮船
- 寻路到港口,随机选择9*9内的格子作为目的地
- 机器人和货物貌似应该绑定,目前会抖动做无用功