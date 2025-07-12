# EDA-SLAM: Efficient Deep Active SLAM via Unknown-Region Reward and Dynamic Termination

![GitHub license](https://img.shields.io/github/license/pyyyz/edaslam)
![GitHub stars](https://img.shields.io/github/stars/pyyyz/edaslam?style=social)

**EDA-SLAM** is a deep reinforcement learning (DRL)-based framework for **Active Simultaneous Localization and Mapping (SLAM)**, designed to improve exploration efficiency, training stability, and real-world deployment performance. Built on **Proximal Policy Optimization (PPO)**, EDA-SLAM introduces three core innovations:

1. **Unknown-region-driven reward function** for sustained exploration at high map coverage.
2. **Dynamic window pose monitoring** to prevent ineffective behaviors (e.g., spinning or getting stuck).
3. **Frontier-driven dynamic termination** to autonomously conclude exploration upon task completion.

This framework achieves **40% faster convergence** compared to baseline methods and reduces **exploration time by 81.5%** and **path length by 41.1%** in real-world deployment.

## 📦 Requirements

- Ubuntu 20.04
- ROS (Robot Operating System)
- Gazebo
- Python 3.8+


---

## 🛠️ Installation

```bash
git clone https://github.com/pyyyz/edaslam.git
cd edaslam
pip install -r requirement.txt
catkin_make
```


---

## 🚀 Usage

### Training

```bash
cd edaslam
source devel/setup.bash
roslaunch drl_agent trainmapping.launch
```

### Testing

```bash
cd edaslam
source devel/setup.bash
roslaunch drl_agent evalmapping.launch
```



### Real-world Deployment

Ensure your robot is equipped with:

- LiDAR (0.1–3.5m)
- Onboard processor (e.g., Intel N100)
- ROS-compatible drivers (e.g., sunray_swarm)

Then run:

```bash
roslaunch sunray_swarm wheeltec_driver.launch
roslaunch turn_on_wheeltec_robot robot_pose_ekf.launch
roslaunch turtlebot3_gazebo turtlebot3_gmapping.launch
python trainGPPO.py
```

---


## 📄 License

This project is licensed under the **MIT License**.
