# Duckiebot — Portfolio-opdracht 2: Odometry + SLAM + EKF

## commands

```bash
# 1. Bouw
dts devel build -H DUCKIEBOT_NAAM -f

# 2. (Optioneel maar aanbevolen) Diagnose eerst
dts devel run -H DUCKIEBOT_NAAM -L diagnostic
# → Geeft elke 3s aan welke topics binnenkomen.
# 3. Start alles
dts devel run -H DUCKIEBOT_NAAM

# 4. Bestuur in andere terminal
dts duckiebot keyboard_control DUCKIEBOT_NAAM
```

## Structuur

```
duckiebot-slam-opdracht2/
├── .dtproject
├── Dockerfile
├── README.md
├── launchers/
│   ├── default.sh                    
│   ├── diagnostic.sh                 
│   ├── odometry.sh
│   ├── slam.sh
│   ├── fusion.sh
│   ├── camera.sh
│   ├── encoders.sh
│   ├── wheel-control.sh
│   └── twist.sh
└── packages/
    └── slam_pkg/
        ├── CMakeLists.txt
        ├── package.xml
        ├── rviz/slam.rviz
        └── src/
            ├── odometry_node.py          
            ├── slam_node.py              
            ├── fusion_node.py            
            ├── path_publisher_node.py
            ├── camera_reader_node.py
            ├── wheel_encoder_reader_node.py
            ├── wheel_publisher_node.py
            ├── twist_publisher_node.py
            └── diagnostic_node.py        
```

## Launchers

Elke launcher start één node los — handig voor testen.

| Launcher | Doet |
|----------|------|
| `default` | Start odometry + slam + fusion + path (ALLES) |
| `diagnostic` | Print elke 3s welke topics binnenkomen |
| `odometry` | Alleen Taak 1 |
| `slam` | Alleen Taak 2 |
| `fusion` | Alleen Taak 3 (EKF) |
| `camera` | Camera reader, print resolutie en fps |
| `encoders` | Encoder reader, print ticks |
| `wheel-control` | WASD-toetsenbordbesturing |
| `twist` | Demo chassis (v, ω) commando |

Gebruik:
```bash
dts devel run -H DUCKIEBOT -L <launcher_naam>
```

## Topics

| Topic | Type | Producer |
|-------|------|----------|
| `/<veh>/odometry/pose` | `nav_msgs/Odometry` | `odometry_node` |
| `/<veh>/odometry/pose_stamped` | `geometry_msgs/PoseStamped` | `odometry_node` |
| `/<veh>/odometry/path` | `nav_msgs/Path` | `path_publisher_node` |
| `/<veh>/slam_node/features/compressed` | `sensor_msgs/CompressedImage` | `slam_node` |
| `/<veh>/slam_node/map/compressed` | `sensor_msgs/CompressedImage` | `slam_node` |
| `/<veh>/slam_node/landmarks` | `geometry_msgs/PoseArray` | `slam_node` |
| `/<veh>/slam_node/correction` | `std_msgs/Float32MultiArray` | `slam_node` |
| `/<veh>/fusion_node/pose` | `nav_msgs/Odometry` | `fusion_node` |
| `/<veh>/fusion_node/path` | `nav_msgs/Path` | `fusion_node` |


