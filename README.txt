使用“test_trajectory.m”启动程序
文件夹“readonly”：只读文件
  quadModel_readonly.m：500g四旋翼的一些参数
  quadEOM_readonly.p: 四旋翼动力学模型
  run_trajectory_readonly.m: 迭代进行：求解动力方程，接受期望轨迹，运行你的控制器代码，并进行可视化。

文件夹“utils”：一些有用的函数，例如四元数转换。
test_trajectory.m：主要的程序入口。

-----------------------------------------------------------------------------------------------------------
controller.m: 需要编写代码的部分。给定当前和期望的状态向量，计算力和力矩。
***linear_controller.m作为线性控制器的参考答案***

*_trajectory.m: 需要编写代码的部分。根据路径设计四旋翼的轨迹。根据当前的状态向量和时间计算期望的状态向量。 
  hover_trajectory.m悬停轨迹
  diamond trajectory.m和circle_trajectory.m，分别是钻石形和圆形轨迹

