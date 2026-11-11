# Standalone flight programs

These are preserved working programs and candidates, not interchangeable entry
points. Read each program's README and obtain flight approval before execution.

| Directory | Contract and status |
|---|---|
| [flyx_successes](flyx_successes/README.md) | Reported successful native-PX4 FlyX programs; simulator-only `/fmu/in/*` ownership |
| [flyx_course](flyx_course/README.md) | Generated waypoint controller on Sky Beta interface; holds for pilot landing |
| [flyx_skybeta](flyx_skybeta/README.md) | Simulator pilot/interface launcher, not hardware startup |
| [flyx_square](flyx_square/README.md) | Bounded native-PX4 square diagnostic |
| flyx_candidates | Untested variants; presence does not establish qualification |
| [jetson_shadow](jetson_shadow/README.md) | Shadow integration/fake-drone tests |

Some files embed a controller and are generated. Edit their template/builder and
test it; preserve successful generated programs. Newer teammate code is separately
captured under [team](../team/README.md). Overlapping copies retain their context.
