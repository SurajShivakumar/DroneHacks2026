# jetson_shadow: Jolie's v4c planner on the Sky Beta Jetson, in SHADOW

This is a 60-second bench test. It runs Jolie's FlyX-verified planner on the real drone's Jetson
without ever commanding the vehicle, and checks that every topic it needs is flowing and nothing
trips.

| File | What it is |
|---|---|
| `upload_to_flyx_v4_skybeta.py` | The program: a single file (rclpy + numpy; scipy optional). **SHADOW by default**; flying needs `--fly`. |
| `shadow_topic_check.py` | A read-only checker. It reports per-topic rate, max gap and last age, checks the ROS graph, and gives PASS/FAIL. |
| `run_shadow_test.sh` | The one script you run, **on the Jetson host**. |
| `in_container.sh` | A helper that `run_shadow_test.sh` copies into the container. |
| `build_v4_skybeta.py`, `skybeta_shadow_main.py.in` | They regenerate the program from v4c. Development only: they need the repo. |
| `tests/` | Laptop only. `test_shadow.py` holds the unit tests. `mock_drone.py` is the mock drone. `e2e_humble.sh` runs the end-to-end test. `judge_host_pauses.py` re-judges laptop runs. `Dockerfile.humble` builds the test image. |

## What the program is

It is v4c: `origin/jolie-flyx-v2` at **1d44162**, `pathfinding/flyx/upload_to_flyx.py`, sha256
`feb73d6434da…`. That is byte-identical to `flyx_successes/upload_to_flyx_704.py`, the file that
completed FlyX 5/5 (NAVIGATE 119.9 s). Its header says "commit b142eb5" because it was built on
top of b142eb5 before v4c was committed. The file committed **at** b142eb5 is a different one and
did not fly.

The conversion follows `jolie_course/build_skybeta_copy.py`:
- **Kept byte for byte:** every embedded module (`_SOURCES`: mission, planner, planner process),
  the map and the checkpoints. The builder asserts this.
- **Replaced:** only the PX4 I/O. Inputs are `/visual_slam/tracking/odometry`, `/skybeta/state`
  and `/camera/depth/image_raw` with its `camera_info`. The output is `/skybeta/setpoint/pose`,
  and only with `--fly`.
- **No `/fmu`:** the program code mentions `/fmu` in exactly one place, the forbidden-topic guard.

### Shadow ("fake-armed"): the default

- **Never published:** `/skybeta/setpoint/pose`, `/skybeta/setpoint/twist` or any `/fmu/in/*`
  topic, and it makes no service calls. It creates publishers through a single guarded function
  that refuses those topics. Every 2 s it also checks its own publishers in the ROS graph and
  stops if one ever appears.
- **Published instead:** `/jolie_flyx/shadow/setpoint_pose` (the PoseStamped it would command),
  `/jolie_flyx/status` (JSON at 20 Hz) and a `journal.jsonl`.
- **Fake-armed:** it starts only after the **real** vehicle shows fresh odometry, `px4_link`,
  landed and disarmed. It then pretends the pilot armed and the interface entered Offboard, and
  steps through STREAM → WAIT_OFFBOARD → WAIT_ARM → TAKEOFF on real odometry and depth. On the
  bench it stays in TAKEOFF.
- **`--fake-altitude`** (the test uses it): after 3 s in TAKEOFF it adds 0.85 m to the odometry
  z, so NAVIGATE, depth mapping and the planner process all run. This is logged as
  `[SHADOW] FAKE ALTITUDE`.
- **Real vehicle armed:** if `/skybeta/state` ever says `armed` or `offboard_active` while in
  shadow, it logs an `ERROR` ("REAL VEHICLE armed…") every 2 s and keeps publishing nothing.
- **Foreign setpoint publisher:** the organizer's square example, for instance, is logged at INFO
  and tolerated.
- **FAILED mission:** in fake-armed shadow, if the unchanged mission goes FAILED (for example,
  odometry older than 0.3 s during the 1 s STREAM window), it logs a WARN and restarts the shadow
  mission 5 s later, so the rest of the run is still exercised. The WARN still fails the test.
  With `--fly` it exits as before.
- **Stale inputs:** odometry older than 0.5 s, state older than 1.5 s or depth older than 1 s
  produce a `WARN stale input`. A control-loop gap over 0.25 s produces a `WARN … stall`.

## 1. Copy the folder to the Jetson

Use one of the following. Replace `USER@JETSON` with the drone's login, for example the
skybeta-06 address. Type the password when asked. Never put it in a file.

From Windows PowerShell (OpenSSH `scp` is built in):
```
scp -r "C:\Users\Suraj\Documents\Projects\DroneHacks 2026\jetson_shadow" USER@JETSON:~/
```

With PuTTY's `pscp`:
```
pscp -r "C:\Users\Suraj\Documents\Projects\DroneHacks 2026\jetson_shadow" USER@JETSON:/home/USER/
```

With Git, once branch `jetson-shadow` has been pushed. It is local only for now.
```
git clone -b jetson-shadow https://github.com/jakefromstatefarm00/dronehacks2026---simready.git ~/simready
ln -s ~/simready/jetson_shadow ~/jetson_shadow
```

## 2. Run it (on the Jetson host, not inside the container)

First check the bench: drone landed, **disarmed**, props off, `skybeta-stack` up.

```
ssh USER@JETSON
sudo -v                      # optional: lets the test record USB events via 'sudo -n dmesg -w'
bash ~/jetson_shadow/run_shadow_test.sh
```

The test takes about 80 s: preflight, about 10 s for the program to reach TAKEOFF, 5 s of
warm-up, then 60 s of checking. Options are `--duration N`, `--no-fake-altitude` and
`--container NAME`. The script has no way to fly: it refuses `--fly`.

What it does:
1. **Preflight.** It checks that the container is up and that `lsusb` shows `03e7:f63b`.
   `03e7:2485` means the camera is unbooted, and the script refuses. It then copies the files to
   `/tmp/jetson_shadow` in the container. It reads one `/skybeta/state`, and **refuses (exit 2) if
   it reports armed, offboard or not landed**, or if a `jolie_flyx` node is already running.
2. **USB monitor.** It records `dmesg -w`. It uses `sudo -n` if you ran `sudo -v`; if not, it
   skips this and says so. No password is stored.
3. **Program.** It starts the program in the container with
   `--shadow --fake-altitude --depth-topic /camera/depth/image_raw --camera-offset 0.1053 0 0.0304`
   (the stack's TF mount).
4. **Checker.** It runs `shadow_topic_check.py` for 60 s.
5. **Stop and collect.** It stops the program with SIGINT, by PID (SIGTERM, then SIGKILL, only if
   it hangs). Everything goes into `~/team_ws/runs/shadow-<time>/`: `program.log`,
   `checker_report.txt`, `checker.json`, `journal.jsonl`, `usb.log`, `env.txt` and `summary.txt`.
6. **Summary.** It prints a one-screen summary and exits with 0 on PASS, 1 on FAIL or 2 if it
   refused.

## 3. Read the result

The last line is `RESULT: PASS` or `RESULT: FAIL`, and each `FAIL:` line says why. The table above
it has one row per topic:

| key | topic | expected (mean ≥, max gap ≤) |
|---|---|---|
| odometry | /visual_slam/tracking/odometry | 20 Hz, 0.5 s |
| skybeta_state | /skybeta/state | 2 Hz, 1.0 s |
| depth | /camera/depth/image_raw | 15 Hz, 1.0 s (lazy: 5 s warm-up) |
| depth_info | /camera/depth/camera_info | 5 Hz, 1.5 s |
| imu | /oak/imu/data | 100 Hz, 0.25 s |
| sensor_combined | /fmu/out/sensor_combined | 50 Hz, 0.5 s (SKIP if px4_msgs is not importable; diagnostic only) |
| status | /jolie_flyx/status | 15 Hz, 0.5 s |
| shadow_setpoint | /jolie_flyx/shadow/setpoint_pose | 15 Hz, 0.5 s while in STREAM…NAVIGATE |

**PASS means all of the following:**
- Every input arrived at its rate with no long gap.
- The program reached TAKEOFF (and NAVIGATE with fake altitude) and streamed would-be setpoints
  on the shadow topic only.
- The ROS graph never showed our node publishing `/skybeta/setpoint/*` or `/fmu/in/*`.
- The real vehicle never reported armed.
- `program.log` has no `[WARN]`, `[ERROR]`, `Traceback`, `Error:` or `Warning:` lines, so there
  were no stale inputs, stalls or exceptions.
- `journal.jsonl` has no error records.
- No USB disconnect, U1/U2 failure or reset appeared in dmesg, and the OAK is still `03e7:f63b`.
- The program shut down cleanly with exit 0.

Foreign publishers (the flight interface on `/fmu/in`, the square example on
`/skybeta/setpoint/pose`) are listed but do not fail the test.

PASS does not mean the drone can fly the course. The camera mount, the course-frame alignment on
the real launch mark and VIO in flight are untested. Before any `--fly`, the team reviews and
approves the mission.

## 4. Typical FAILs and what to do

| FAIL line | Likely cause | Do this |
|---|---|---|
| `REFUSED: … ARMED / offboard / not landed` | The vehicle is armed, or the state says so | Land and disarm with the RC, then re-run. Never bypass this. |
| `REFUSED: OAK shows 03e7:2485` / `no OAK camera` | The OAK USB link dropped (camera unbooted) | Restart the camera node or stack (organizer procedure) or replug, wait for `lsusb` to show `03e7:f63b`, re-run. |
| `REFUSED: no /skybeta/state` | The flight interface is down, or ROS_DOMAIN_ID / DDS profile is wrong | `docker exec skybeta-stack bash -ic 'ros2 topic hz /skybeta/state'` |
| `odometry: gap … > 0.5` / `stale input: odometry` | cuVSLAM paused or lost tracking (a bare scene, covered camera, CPU load) | Look at the scene. `ros2 topic hz /visual_slam/tracking/odometry`. Check the stall count. |
| `depth: no messages` / `rate … < 15` | The depth relay is down, or the lazy OAK stereo stream did not start | `ros2 topic hz /oak/stereo/image_raw` and `/camera/depth/image_raw`. Check the `depth_relay` node. |
| `imu: rate …` | The OAK IMU is slow, or the USB link is degraded | Check `usb.log`. Replug and re-run. |
| `program did not reach TAKEOFF` | WAIT_HEALTH never passed (stale odometry, no `px4_link`, `local_enu` missing, not landed) | `tail program.log` and the `note` in `/jolie_flyx/status`. |
| `shadow_setpoint: … never reached a streaming state` | Same as above | Same as above |
| `N WARN/ERROR/Traceback line(s)` | Read them in the summary grep | `stall` means CPU load (run `top`). `slow planner step` means the planner is overloaded. A Traceback is a bug: send `program.log`. |
| `program publishes vehicle topics` / `SHADOW VIOLATION` | Must never happen | Stop. Do not fly. Send the run folder. |
| `REAL vehicle reported armed/offboard during the check` | Someone armed during the test | Land and disarm. The run is invalid. |
| `USB disconnect/U1/U2/reset event(s)` / `OAK not 03e7:f63b after the test` | The OAK link dropped during the test | Check the cable and the hub. Re-run. A recurring drop is a hardware issue to raise with the organizers. |
| `program exit code …` / `had to be SIGKILLed` | The shutdown hung | Send `program.log`. Check for a leftover process with `ps aux \| grep upload_to_flyx` in the container. |
| `2 nodes named 'jolie_flyx'` | An old copy is still running | Stop it (by PID) and re-run. |

## Developer notes

- **Regenerate:** `python3 jetson_shadow/build_v4_skybeta.py` (from the repo root). Add
  `--source flyx_successes/upload_to_flyx_704.py` to build from the flown file. It checks the
  sha256 and that `_SOURCES` are equal.
- **Tests (laptop, Humble image):**
  ```
  docker build -t jetson-shadow-humble -f jetson_shadow/tests/Dockerfile.humble jetson_shadow/tests
  MSYS_NO_PATHCONV=1 docker run --rm -v "$(cygpath -w "$PWD")":/repo -e WS=/repo/jetson_shadow jetson-shadow-humble bash /repo/jetson_shadow/tests/e2e_humble.sh all
  ```
  This runs the unit tests, then three end-to-end runs of `run_shadow_test.sh --dry-run` against
  `tests/mock_drone.py`: PASS, FAIL on a 3 s odometry gap, and REFUSED when armed.
- **Laptop caveat.** On the Windows laptop, the WSL2 / Docker Desktop VM freezes every process for
  0.4–1.5 s every 20–30 s. The mock measures this itself and logs `MOCK PAUSE`, and it happens
  with no program running. A 60 s window almost always contains one, so the raw laptop verdict
  is FAIL. `tests/judge_host_pauses.py` then re-judges the run: it excuses only gaps and WARN
  lines that overlap a measured pause, and everything else stays a FAIL. The injected 3 s
  odometry gap is never excused. The judge is laptop-only. On the drone, `run_shadow_test.sh` is
  the verdict and excuses nothing. A real stall there is a real finding.
