# sep16

Open-loop handle grasping for a Reach Alpha 5 arm on ROS Melodic.

The vehicle parks in front of the mine. One command starts the pick: the arm collects a few camera frames, finds the handle, plans a collision-free path and then drives to the handle blind. The arm blocks the camera during the move, so nothing is corrected on the way.

## Pipeline

```
/task/start
   │
   ▼
COLLECT ──► PROCESS ──► PLAN ──► EXECUTE ──► JAWCLOSING ──► DONE
   ▲                      │
   └──────── RETRY ◄──────┘   no candidate or no path: wait, collect again
```

| Stage | Node | What happens |
|---|---|---|
| Collect | `cloud` | Pairs `/pointcloud` with `/grasp_poses` by stamp and gathers N frames |
| Process | `cloud` | Crops the cloud to the arm's reach, builds a voxel obstacle map, separates handle from rope, averages across frames and publishes candidates |
| Plan | `planner` | Runs inverse kinematics per candidate, then bi-directional RRT* with forward-kinematics collision checks, and keeps the path with the lowest weighted joint travel |
| Execute | `executor` | Sends the path in small joint steps and stops if a joint stops following |
| Grip | `task` | Closes the jaw and reports whether something was held; the pick counts as done either way |

`/task/stop` stops everything and releases the arm. Any fault also ends in standby.

## Packages

| Package | Role |
|---|---|
| `bringup` | The launch file and every config yaml. The only place to change behaviour |
| `msgs` | Messages and the Collect, Plan and Execute actions |
| `params` | Strict parameter reader: a missing or invalid key stops the node with a clear message |
| `description` | URDF, meshes and the camera mount |
| `kine` | Arm model, forward and inverse kinematics, collision body |
| `driver` | BPL serial protocol to the arm, or a simulated arm |
| `cloud` | Point cloud to obstacle map and grasp candidates |
| `planner` | Candidates to a joint path |
| `executor` | Joint path to arm motion |
| `task` | State machine that runs the pick |
| `viz` | All Foxglove markers in one node |

The handle classifier and the averaging method are both chosen in `cloud.yaml` (`method:`), so either can be swapped without touching the pipeline.

## Build and run

Everything runs in the Melodic dev image. The workspace must be mounted at the same path inside and outside the container.

```bash
scripts/build.sh                 # build every package
scripts/build.sh planner cloud   # build some
scripts/shell.sh                 # shell in the container, ROS sourced
```

In that shell:

```bash
# simulated arm, recorded camera
roslaunch bringup bringup.launch sim:=true bag:=$PWD/data/rosbag/captures_grasp_poses.bag

# real arm, live camera
roslaunch bringup bringup.launch

rosservice call /task/start
rostopic echo /task/state
rosservice call /task/stop
```

Foxglove connects on `ws://localhost:8765`.

## Switches

All switches are launch arguments. Every number lives in yaml.

| Argument | Default | Effect |
|---|---|---|
| `sim` | false | Simulated arm instead of the serial port |
| `bag` | "" | Replay a bag in a loop instead of the live camera |
| `task` | true | Start the state machine. Off: drive the actions by hand |
| `viz` | true | Start the viz node |
| `foxglove` | true | Start foxglove_bridge |
| `outlier_filter` | true | Drop flying pixels |
| `handle_classifier` | true | Off: every grasp pose is treated as handle |
| `handle_carving` | true | Let the blades close in on the handle surface |
| `corridor_carving` | true | Clear the approach corridor in front of each handle pose |
| `candidate_averaging` | true | Keep only candidates the frames agree on |
| `obstacle_averaging` | true | An obstacle must be seen in several frames |

Frames per attempt are `frames_to_collect` when either averaging switch is on, otherwise 1.

## Config

| File | Holds |
|---|---|
| `topics.yaml` | Every topic, service and action name |
| `arm.yaml` | Link geometry, joint and jaw limits, joint convention, home pose |
| `jaws.yaml` | Jaw geometry, blade profile, open width |
| `camera.yaml` | Intrinsics and the camera mount on the arm |
| `driver.yaml` | Serial port, device ids, polling, home ramp, simulated arm |
| `cloud.yaml` | Voxel size, filters, classifier, averaging |
| `planner.yaml` | Collision model, grasp depth, cost weights, RRT* |
| `executor.yaml` | Speed, tolerances, blocked detection |
| `task.yaml` | Retry delay, attempt limit, grip check |
| `viz.yaml` | Marker sizes, colours, redraw rate |
| `foxglove.yaml` | Bridge port and address |

## Conventions

- **Units:** radians and metres on every topic, with vendor joint angles as reported by the arm. Yaml uses degrees and millimetres where the key name says so (`_deg`, `_m`).
- **Joint model:** kinematic angle = `direction_sign * (reported - zero_offset)`, set in `arm.yaml`.
- **Frames:** `arm_base` → `camera_mount` → `camera_link`, published from the URDF. Vision publishes in `camera_link`: x right, y up, looking along -z.
- **Home:** shoulder 90° with the jaw open. The shoulder below about 80° drives the upper arm into the base housing.
- **Collision:** links are capsules; the jaw blades are sampled from their real profile. Unseen space counts as free. The safety floor is `arm_base` z = 0.

## What limits success

Measured on the capture bag, in simulation:

- **The scene must stay still.** A plan is always clear when it is made. After 1 s of recorded handle motion only about half the plans still clear the obstacles, and arrival takes 20 to 60 s at the default speed. Station keeping matters more than anything in this repo.
- **Approach angle matters.** Square on to the handle, 10% of frames give a plan. Looking down at it from about 60° above, slightly further back, gives 83%. Turning the heading around the handle does not help.
- **Voxel size:** 2.5 mm instead of 5 mm raises head-on planning from 15% to 87% for about 0.1 s more processing.
- **Time goes to waiting, not compute.** Processing takes about 0.1 s and planning about 1 s. Frame collection, retry delay and execution speed (`executor.yaml`) set the pace.
