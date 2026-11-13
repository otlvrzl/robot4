# Robot4: a wheeled robot with a LIDAR exploring a room in Gazebo (ROS 2 Jazzy + Docker)

A small differential-drive robot (box chassis, two wheels, a caster and a LIDAR on top) in a
simulated **8 x 6 m room** with a few obstacles. A Python node (`wander`) makes it drive around
and turn away from walls and obstacles using only the LIDAR. ROS 2 Jazzy, Gazebo Harmonic and
RViz run inside Docker, so nothing but Docker is installed on your Ubuntu host. Your code stays
in `./src` on the host, ready for git/GitHub.

**Project stages**
1. Room in Gazebo + robot that wanders around and avoids obstacles  (done)
2. SLAM (`slam_toolbox`): build a map of the room and save it  <- *this version*
3. Later: Nav2 on the saved map, and a simulated camera for computer vision

## 1. Requirements on the host
- Docker Engine with the Compose plugin (`docker compose version` should work)
- A normal desktop session (the Gazebo and RViz windows appear on your screen)

## 2. First run
```bash
bash setup.sh                 # creates .env, lets Docker open windows on your display
docker compose build          # downloads ROS + Gazebo, takes a while the first time
docker compose up sim         # builds your package, then starts everything
```
You should see Gazebo with the room and RViz. After about 5 seconds the robot appears in the
middle of the room and starts driving. Stop with `Ctrl+C`, then `docker compose down`.

> The image is now called `robot4-ros:jazzy`, so the first `docker compose build` runs again
> (it is quick if Docker still has the cached layers from the old project).

### What you should see
- **Gazebo**: the room with 4 grey walls, an orange box, a green box, a purple column and a
  short partition wall sticking out of the south wall.
- **RViz**: the robot, the LIDAR points (they should line up with the walls and obstacles)
  and the odometry trace (the path the robot has driven).
- **Terminal**: every state change of the robot, for example
  `FORWARD -> TURN: obstacle ahead (0.68 m), turning left (free space: left 2.4 m, right 0.9 m)`.

### Change settings without editing code
```bash
LINEAR_SPEED=0.2 docker compose up sim                    # drive slower
DRIVER=none docker compose up sim                         # nobody drives: you steer yourself
WORLD=ground_world DRIVER=avoider docker compose up sim   # the old task: circle + one box
```

## 3. How the `wander` node thinks
It is a small **state machine** (see `src/simple_bot/simple_bot/wander.py`):

| State | What the robot does | Switches when |
|---|---|---|
| `FORWARD` | Drives ahead (slower near obstacles) and steers gently away from walls at its sides | something closer than 0.6 m in front -> `TURN`; closer than 0.35 m -> `BACK_UP`; every 10-25 s a short random turn for variety -> `TURN` |
| `TURN` | Rotates on the spot towards the side with more free space (random side if both are equal) | the front is free again -> `FORWARD`; turning > 8 s -> `BACK_UP` |
| `BACK_UP` | Reverses a little | after 1.5 s, or a wall behind -> `TURN` |

The random choices (turn side, variety turns) make sure the robot does not repeat exactly the same path forever.

Limits: the robot only reacts to what the LIDAR sees *right now*. It has no map and no plan, so
it explores "randomly". In a room this size a few minutes are usually enough to see most of it.
It does not notice if a wheel is stuck.

Tuning knobs (ROS parameters of `wander`): `linear_speed`, `turn_rate`, `slow_distance`,
`turn_distance`, `emergency_distance`, `side_distance`, `steer_gain`. Their default values (with
comments) are at the top of `Wander.__init__` in `wander.py`: change a number, restart
`docker compose up sim` and watch what happens. A good experiment is a bigger `turn_distance`
(the robot turns earlier) or a smaller one (it comes closer to walls).
To look at them while the simulation runs:
```bash
docker compose run --rm shell
ros2 param list /wander
ros2 param get /wander turn_distance
ros2 topic echo /cmd_vel               # what the wander node commands right now
```

## 4. Stage 2: build a map with SLAM

**What is SLAM?** The robot does not know the room. While it drives, SLAM Toolbox compares
every new LIDAR scan with the earlier ones, corrects the robot's position, and draws walls and
obstacles into a map. It publishes the map on `/map` and the transform `map -> odom` (the
position correction). RViz shows the result live.

### Run it
The Dockerfile has two new packages (`slam_toolbox`, `nav2_map_server`), so rebuild the image once:
```bash
docker compose build
docker compose up mapping      # Gazebo + robot + wander + SLAM, RViz opens with the map
```
In RViz you should see the map grow: **white** = free floor, **black** = walls and obstacles,
**grey** = not seen yet. The red LIDAR points should lie on the black lines. SLAM starts about
6 seconds after the simulation (it waits for the robot to appear).

Let the robot wander until the whole room is white inside and the walls are closed black lines.
In a room this size that usually takes a few minutes. If a corner stays grey, wait a bit longer
or steer the robot there yourself: `DRIVER=none docker compose up mapping`, then in a second
terminal `docker compose run --rm shell` and
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
Driving slowly (`LINEAR_SPEED=0.2`) usually gives a cleaner map.

### Save the map
When the map looks complete, open a **second terminal** (leave `mapping` running) and run:
```bash
docker compose run --rm save_map
# another name: MAP_NAME=room_map_v2 docker compose run --rm save_map
```
This creates two files in `src/simple_bot/maps/` (on your host, because `./src` is shared):
- `room_map.pgm`: the map as a grayscale picture. Open it with any image viewer to check it.
- `room_map.yaml`: the map's settings: which picture to load, `resolution` (metres per pixel,
  0.05 = 5 cm) and `origin` (where the picture's lower-left corner is in the world).

Both files are small, so **commit them to git**. Nav2 will load `room_map.yaml` in a later stage.

### How good is the map?
The room is 8 x 6 m. At 5 cm per pixel the picture should be about 160 x 120 pixels of room plus
a small grey border. Measure a wall in the picture (pixels x 0.05 = metres) and compare. Gazebo's
odometry has no noise, so the map should be very accurate; real robots need more care.

### Look inside SLAM
```bash
docker compose run --rm shell
ros2 topic list                          # now includes /map
ros2 topic hz /map                       # the map is republished every few seconds
ros2 run tf2_tools view_frames           # TF tree: map -> odom -> base_link -> lidar_link
ros2 lifecycle get /slam_toolbox         # should say: active
```
The SLAM settings are in `config/slam_toolbox.yaml` (every important block is explained there).

## 5. Everyday commands
```bash
docker compose run --rm shell        # a terminal inside the ROS environment
```
Inside it (while `sim` is running in another terminal):
```bash
ros2 topic list
ros2 topic echo /odom --once         # where the robot thinks it is
ros2 topic hz /scan                  # LIDAR should run at about 10 Hz
ros2 run tf2_tools view_frames       # draws the TF tree to a PDF
rqt_graph                            # shows which nodes talk to which
```
Drive it yourself with the keyboard (start `sim` with `DRIVER=none` first):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 6. Edit code on the host, run in Docker
Edit files in `src/simple_bot/` with any editor (VS Code, etc.) on your host. Every
`docker compose up sim` rebuilds the package first, so just restart after changes.

## 7. Put it on GitHub
Run these on the host, inside this folder:
```bash
git init
git add .
git commit -m "Initial robot4 project"
# create an empty repository "robot4" on github.com first, then:
git remote add origin git@github.com:YOUR_USER/robot4.git
git branch -M main
git push -u origin main
```
`.gitignore` already excludes `.env`, build output, Python caches and editor files. The saved map
(`maps/*.pgm`, `maps/*.yaml`) is kept on purpose; only the big SLAM "posegraph" files are ignored.
`.env.example` shows what a `.env` looks like (`setup.sh` creates the real one).

## 8. Folder layout
```
robot4/
├── README.md
├── Dockerfile                 # builds the ROS 2 + Gazebo + RViz image
├── docker-compose.yml         # how to run it (shares ./src with the container)
├── ros_entrypoint.sh          # loads ROS when the container starts
├── setup.sh                   # one-time host helper (.env + display access)
├── .env.example               # template for the personal .env (the real .env is not in git)
├── .gitignore                 # files git must not store
├── .dockerignore              # files Docker must not copy when building
└── src/                       # YOUR CODE (this is the folder mounted into Docker)
    └── simple_bot/            # the ROS 2 package
        ├── package.xml        # package name and dependencies
        ├── setup.py           # install rules + program names
        ├── setup.cfg
        ├── resource/simple_bot
        ├── simple_bot/        # Python code
        │   ├── __init__.py
        │   ├── wander.py             # NEW: explores the room, avoids obstacles
        │   ├── circle_driver.py      # plain circle, ignores obstacles
        │   └── obstacle_avoider.py   # circle + LIDAR obstacle avoidance
        ├── launch/
        │   ├── sim.launch.py         # Gazebo + robot + driver + RViz
        │   └── mapping.launch.py     # NEW: sim.launch.py + SLAM Toolbox
        ├── urdf/simple_bot.urdf
        ├── worlds/
        │   ├── room_world.sdf        # NEW: 8 x 6 m room with obstacles
        │   └── ground_world.sdf      # ground + one box (old circle task)
        ├── config/
        │   ├── bridge.yaml
        │   └── slam_toolbox.yaml     # NEW: SLAM settings
        ├── maps/                     # NEW: saved maps (room_map.pgm + room_map.yaml)
        └── rviz/
            ├── sim.rviz
            └── mapping.rviz          # NEW: layout that shows the map
```
(`.gitignore` and `.dockerignore` start with a dot, so file managers hide them: press Ctrl+H.)

## 9. How it works
```
wander --/cmd_vel--> ros_gz_bridge --> Gazebo DiffDrive plugin --> wheels turn
Gazebo LIDAR --> ros_gz_bridge --/scan--> wander (decides where to go) and RViz
Gazebo DiffDrive --> /odom, /tf (odom -> base_link) --> RViz
Gazebo wheel angles --> 2nd bridge --/joint_states--> robot_state_publisher
URDF --> robot_state_publisher --> /tf (base_link -> wheels, lidar)
Stage 2:  /scan + odom->base_link TF --> slam_toolbox --> /map and TF map->odom --> RViz
          /map --> map_saver_cli --> maps/room_map.pgm + room_map.yaml
```

## 10. Troubleshooting
- **No windows / "cannot open display"**: run `bash setup.sh` again, and check `echo $DISPLAY`.
- **OpenGL errors or black windows**: put `SOFTWARE_RENDER=1` in `.env` (slower but works anywhere).
- **`/dev/dri` not found**: delete the `devices:` lines in `docker-compose.yml`.
- **Robot does not appear / "world not found"**: the world name inside the `.sdf` file must equal
  the file name (`room_world.sdf` -> `<world name="room_world">`).
- **Robot appears but does not move**: check the terminal for `Wander started` and run
  `ros2 topic hz /scan`. No `/scan` means no driving.
- **RViz says a frame for the scan does not exist**: run `ros2 topic echo /scan --once` and check
  `frame_id`. It should be `lidar_link`.
- **No `/map` in RViz (Stage 2)**: check that `ros2 lifecycle get /slam_toolbox` says `active`, that
  `ros2 topic hz /scan` shows data, and read the terminal for `slam_toolbox` errors. Messages about
  a missing transform in the first seconds are normal; they stop once the robot has appeared.
- **Map looks smeared or doubled**: the robot probably turned fast while the scan was taken. Use
  `LINEAR_SPEED=0.2`, or lower `turn_rate` in `wander.py`.
- **`save_map` prints "Failed to spin map subscription"**, or `ros2 topic list` in the `shell`
  container shows only `/parameter_events` and `/rosout`: the second container cannot see the
  topics of the `mapping` container. `docker-compose.yml` fixes this with `ipc: host` and
  `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`; after changing them run `docker compose down` and start
  `mapping` again. Meanwhile you can save the map from inside the running container:
  `docker compose exec mapping bash -c "source /opt/ros/jazzy/setup.bash && ros2 run nav2_map_server map_saver_cli -f /ws/src/simple_bot/maps/room_map"`
- **Permission denied in /ws/build**: run `docker compose down -v` and start again.

## 11. Next steps
1. Stage 3 (later): Nav2 loads the saved map and drives the robot to goals you click in RViz.
2. Add a simulated camera for computer vision.
