my_robot_control

Ein ROS 2‑Paket zur kartesischen Steuerung eines Panda‑Roboters (Franka Emika) mit MoveIt 2. Es basiert auf der offiziellen MoveIt 2 Getting‑Started‑Demo und erweitert diese um eigene Commander‑Skripte sowie ein Werkzeug zur Trajektorien‑Visualisierung.

Inhaltsverzeichnis

Voraussetzungen

Installation

Build & Setup

Wichtige Befehle

Paketübersicht

Beispiele

Tipps für WSL 2

Weiterführendes

Lizenz

Voraussetzungen

Betriebssystem: Ubuntu 24.04 LTS unter Windows Subsystem for Linux 2 (WSL 2) ✻(natürlich läuft das Paket auch auf nativem Linux; nur RViz‑GUI erfordert X11/Wayland)

ROS 2 Distribution: Jazzy Jenkins

MoveIt 2: installiert für Jazzy (apt oder Quellcode)

Python ≥ 3.12

Benötigte ROS‑Pakete werden automatisch via rosdep aufgelöst, etwa

sudo apt install ros-jazzy-desktop-full ros-jazzy-moveit-full

Installation

# Workspace anlegen
mkdir -p ~/ws_my_robot/src && cd ~/ws_my_robot/src

git clone <DEIN_FORK_ODER_REPO>/my_robot_control.git

# Abhängigkeiten auflösen
cd ~/ws_my_robot
rosdep install --from-paths src -y --rosdistro jazzy

Build & Setup

cd ~/ws_my_robot
colcon build --packages-select my_robot_control

# Workspace einrichten (jedes neue Terminal!)
source install/setup.bash

Wichtige Befehle

Zweck

Befehl

Arm in Home‑Pose bringen

ros2 run my_robot_control home_commander

Kartesische Ziel‑Pose anfahren (x y z qx qy qz / yaw‑only‑Variante)

ros2 run my_robot_control cartesian_commander 0.2 0.3 0.2 0 0 1.57

Trajektorie live aus RViz 2 oder Bag‑File visualisieren

ros2 run my_robot_control trajectory_visualizer

Demo‑Launch mit MoveIt Task Constructor

ros2 launch moveit_task_constructor_demo demo.launch.py

Alle Befehle setzen voraus, dass das vorherige source install/setup.bash ausgeführt wurde.

Paketübersicht

Datei / Ordner

Zweck

cartesian_commander.py

Liest Zielpose von CLI‑Argumenten, plant kollisionsfreie Trajektorie mit MoveIt 2 und sendet sie an den Panda‑Arm.

home_commander.py

Fährt den Roboter in eine vordefinierte Home‑Position.

trajectory_visualizer.py

Subscribt display_planned_path, extrahiert Joint‑Trajektorien und zeigt Winkel‑vs‑Zeit‑Plots (matplotlib).

launch/

Beispiel‑Launchdateien, z. B. panda_demo.launch.py.

Beispiele

Pick‑&‑Place Mini‑Flow

ros2 launch my_robot_control panda_demo.launch.py execute:=false
# zweites Terminal
ros2 run my_robot_control cartesian_commander 0.5 0.0 0.25 0 0 0
ros2 run my_robot_control cartesian_commander 0.5 0.2 0.15 0 0 1.57
ros2 run my_robot_control home_commander

Trajektorienanalyse aus Bag‑File

ros2 run my_robot_control trajectory_visualizer --bag ~/logs/latest

Tipps für WSL 2

Echtzeit‑Performance: Deaktiviere Windows Fast Startup und prüfe .wslconfig (CPU‑Kerne & RAM).

GUI: WSLg unter Ubuntu 24.04 enthält Wayland‑Server – RViz 2 startet ohne zusätzliche X‑Server.

USB‑Passthrough: Für echte Panda‑Hardware empfiehlt sich Linux‑Dual‑Boot oder eine VM mit PCIe‑Passthrough.

Weiterführendes

MoveIt 2 Tutorials

Franka Control Interface

ROS 2 Jazzy Doku

Lizenz

Apache 2.0 – siehe LICENSE.