## 接口

```xml
  <node name="rb2" pkg="cluster_formation" type="dwa.py" output="screen" >
    <param name="selfname" value="rb2" />
    <param name="target" value="rb1 1.0 0.0" />
  </node>
```

- 领航者命名空间：rb1
- 跟随者命名空间：rb2
- target：领航者相对跟随者距离，rb1在rb2前方1.0米
- 订阅话题：rb2/scan, rb2/odom, rb1/odom
- 发布话题：rb2/cmd_vel

