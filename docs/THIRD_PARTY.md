# Attribution and source boundaries

This repository consolidates team work and research experiments. It does not
claim all included code or assets as original work by one author. No blanket
license is assigned to third-party content; retain applicable upstream terms.

| Component | Source / attribution |
|---|---|
| Team source snapshot | [jakefromstatefarm00/dronehacks2026---simready](https://github.com/jakefromstatefarm00/dronehacks2026---simready), revision `384eedd4ee728d5802a6dbde8ba69406080694a7`; tracked-source hashes in `team/SOURCE_MANIFEST.json` |
| Earlier RL baseline/course loaders | [jakefromstatefarm00/dronehacks2026](https://github.com/jakefromstatefarm00/dronehacks2026); historical experiments reference separately pinned local checkouts |
| Online planner | Jolie's Theta*/A* planner in `team/pathfinding` and its ROS package copy; preserve authorship and package metadata |
| Waypoint controller embedded by builders | [machmoon/dronehacks-sim](https://github.com/machmoon/dronehacks-sim), `mission/core.py`; generated files/builders record source revisions |
| Organizer stack | [Artash-N/skybeta](https://github.com/Artash-N/skybeta), inspected local fleet-ready snapshot, version 0.3.2 / `baf21fd`; not vendored here |
| Course geometry | Organizer-supplied `dronehacks-course-v3.obj` and `.mtl`; supplied assets retain their original ownership |
| Aircraft CAD | Organizer-supplied Sky Beta CAD, excluded from Git; derived dynamics include documented engineering estimates |
| Estimators/runtime dependencies | OpenVINS, NVIDIA cuVSLAM/Isaac ROS, PX4, ROS 2, Gazebo and MuJoCo remain separate upstream projects with their own terms |

The imported team snapshot excludes recorded drone data, nested Git worktrees,
firmware binaries and untracked local edits. Source headers and existing package
license declarations are preserved. A package's declared license does not grant
new rights over unrelated organizer assets or embedded external sources.

See `docs/MODEL_BENCHMARKS.md` for VLA model/checkpoint attribution. Model weights
are not distributed in this repository. Check upstream licenses before any public
redistribution; the initial GitHub repository is private.
