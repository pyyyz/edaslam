# Efficient Autonomous Exploration with Adaptive Deep Reinforcement Learning and Intelligent Termination

![GitHub license](https://img.shields.io/github/license/pyyyz/edaslam)
![GitHub stars](https://img.shields.io/github/stars/pyyyz/edaslam?style=social)

Autonomous exploration remains a fundamental challenge in robotics, particularly when balancing comprehensive environmental coverage efficiency. Current Deep Reinforcement Learning approaches suffer from training instability due to sparse rewards, inefficient termination mechanisms, and suboptimal reward designs that compromise exploration completeness. This paper introduces a novel Deep Reinforcement Learning framework for efficient autonomous exploration, which reimagines the process as holistic and adaptive, guided by environmental feedback and intelligent termination criteria. Our approach features three synergistic innovations: an unknown-region-driven reward function that adaptively scales incentives as environments become increasing mapped; a sliding window pose monitoring system that detects and terminates ineffective behaviors such as repetitive rotation or oscillation; and a frontier-driven termination mechanism that autonomously recognizes task completion through topological analysis of map frontiers. Validated through extensive simulation across diverse environments and rigorous real-world deployment in challenging laboratory settings with narrow corridors and dense obstacles, our framework demonstrates superior exploration efficiency while maintaining robust sim-to-real transfer without costly fine-tuning.Experimental results show our approach achieves $81.5\%$ faster exploration time and $41.1\%$ shorter path lengths, establishing a new paradigm for efficient autonomous exploration in time-sensitive applications.

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
