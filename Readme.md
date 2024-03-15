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


- 本地,评测机可能有两种状态,如何更新状态?
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

