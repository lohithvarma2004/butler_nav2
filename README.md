
# Mobile Differential Drive Robot – Autonomous Order Delivery System (ROS 2)

This project demonstrates an **autonomous differential drive robot** using **ROS 2 Humble**, integrated with **Nav2**, to simulate order delivery to different tables. The robot:

- Accepts delivery orders via a `/order` topic.
- Queues them and follows a priority-based FIFO execution.
- Cancels ongoing or queued orders via `/cancel` topic.
- Waits for user confirmation at target locations via `/confirmation_topic`.
- Uses `nav2_bringup` for localization and navigation.
- Is fully testable in simulation with Gazebo.

---

## 📦 Requirements
- ROS 2 Humble
- `mobile_dd_robot` package
- Navigation2 stack (`nav2_bringup`)
- A saved map (`my_map.yaml`) and proper nav parameters

---

## 🚀 Launch Instructions

### 1. Launch Localization (AMCL)
```bash
ros2 launch nav2_bringup localization_launch.py use_sim_time:=true \
params_file:=/home/lohithvarma2004/assignment/src/mobile_dd_robot/config/nav2_params.yaml \
map:=/home/lohithvarma2004/assignment/src/mobile_dd_robot/config/my_map.yaml
```

### 2. Launch Navigation
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
params_file:=/home/lohithvarma2004/assignment/src/mobile_dd_robot/config/nav2_params.yaml
```

---

## 🧠 Sending Orders

Send up to 3 delivery orders in a queue. Example:
```bash
ros2 topic pub /order std_msgs/String "data: 'table_1'" --once
ros2 topic pub /order std_msgs/String "data: 'table_2'" --once
ros2 topic pub /order std_msgs/String "data: 'table_3'" --once
```

---

## ❌ Canceling Orders

Cancel all orders (queued or active):
```bash
ros2 topic pub /cancel std_msgs/String "data: 'table_1'" --once
ros2 topic pub /cancel std_msgs/String "data: 'table_2'" --once
ros2 topic pub /cancel std_msgs/String "data: 'table_3'" --once
```

Cancel with a shell one-liner:
```bash
for t in table_1 table_2 table_3; do ros2 topic pub /cancel std_msgs/String "data: '$t'" --once; done
```

---

## ✅ Confirming Delivery

Send confirmation **only when robot is waiting** at a location:
```bash
ros2 topic pub /confirmation_topic std_msgs/String "data: 'kitchen'" --once
ros2 topic pub /confirmation_topic std_msgs/String "data: 'table_1'" --once
ros2 topic pub /confirmation_topic std_msgs/String "data: 'table_2'" --once
ros2 topic pub /confirmation_topic std_msgs/String "data: 'table_3'" --once
```

---

## 🎥 Demonstration Video

Watch the full working demo here:  
👉 (https://drive.google.com/file/d/1id1qKYSVLW--t-nNr-veBlwH1VoH8pI2/view?usp=sharing)

---

## 📁 Repo Structure

```
mobile_dd_robot/
├── launch/
│   └── robot_launch.py
├── config/
│   ├── nav2_params.yaml
│   └── my_map.yaml
├── maps/
├── src/
│   └── (custom nodes)
```

---

## 📜 License
MIT License. Free to use and modify.
