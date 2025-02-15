# UAV Linar Control for matlab

本仓库为经典四旋翼无人机的线性控制在matlab中的仿真代码。
本仓库代码中使用的控制器是一个串联线性控制器，位置环作为外环，姿态环作为内环。依托PD+前馈+模型的原理来设计。


一般而言，对于四旋翼飞行器的建模，主要包括刚体运动学模型、刚体动力学模型、控制效率（分配）模型和动力单元模型四个部分。
在上层控制（多旋翼飞行器控制刚体模型）中我们是做到力和力矩的层次。然后在PX4飞控中根据控制分配模型，给各个电机分配期望转速。最后根据动力单元模型，去计算所需的油门。
**不在PX4中直接使用位置命令和速度命令**

![c1876e1aaf98a99124fc37c62c19c5d](https://github.com/user-attachments/assets/92d4e32f-cd78-46af-8d74-936d018ccab9)


MPC：其实更像一个规划器，规划接下来一段时间，每间隔 $\Delta T$为一个状态，规划每个状态应该取什么，使系统指标尽可能小。相比于多项式轨迹规划，MPC规划的就是一个个状态量，状态量之间存在关系。

Planning：开环的Control。已知系统模型，预测系统输入之后，系统会演化成什么样的轨迹。
Control：闭环的Planning。在实际运行中，通过设计合适的控制器去弥补模型的mismatch和外部扰动。

更多先进控制器的话，可参考[Geometric Tracking Control of a Quadrotor UAV on SE(3)](https://arxiv.org/abs/1411.2986) 和 [Minimum Snap Trajectory Generation and Control for Quadrotors](https://ieeexplore.ieee.org/document/5980409)，其中介绍的SE(3)控制器是之后学界主流运用的四旋翼控制的源头。


知识来源为[无人机系统 - 高飞组超全讲解！2021ROS暑期学校 第五天精剪回放](https://www.bilibili.com/video/BV1Jq4y1T7QD?spm_id_from=333.788.videopod.episodes&vd_source=d59d7bc22106d87da35c63b8af6491e8&p=3) 和 北航全权老师的《多旋翼飞行器设计与控制》
