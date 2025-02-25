## 基于卡尔曼滤波获取Pitch角度制作的平衡车

---
### 使用了C++的方式封装，方便后续对各个模块如PID，520电机，以及KLM的抽离使用
---
### 几个关键问题   
  1.  机械中值：小车的机械中值决定了小车的平衡位置，机械中值即车子直立启动时能保持平衡的直立位置。如果机械中值确定错误，则小车在调直立环的时候可能会一直前倾，不好调。原因可能是PID输出为零的位置小车却仍有较大的前倾趋势。**同时调直立环的时候要尽量保持上电姿态一致（与确定机械中值时的）**
  ---
  2.  实际角度获取：如果使用了卡尔曼滤波，注意对加速度与陀螺仪置信度的选取，以及陀螺仪模型的更新周期。确定不好可能导致多方面问题。可能一直无法调出各种视频教程中的现象。  
    **调整方法：** 更新周期可适当选的比定时器中断长一些，如本人代码里，定时器周期是5ms，算上代码运行时间7ms，给到了10ms的更新时间。还有，陀螺仪置信度应较高，加速度计应较小，至少波形上不要出现大于5度的超调（感觉）。因为陀螺仪实际上记的比较准，但是有累计误差，加速度计作为辅助修正。

