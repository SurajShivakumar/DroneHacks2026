# Selected benchmark evidence

These are copies of existing reports/evaluations, not new flights performed during
repository cleanup. Original bytes and local source paths are recorded in
[SOURCE_MANIFEST.json](SOURCE_MANIFEST.json). Raw evidence and videos remain in
local ignored experiment storage.

| Report | Result |
|---|---|
| [v83](gb10-sky-beta-course-v83/README.md) | 113.85 s simulated arm-to-disarm, full native OpenVINS course; one completion in three same-batch attempts |
| [v55](gb10-sky-beta-course-v55/README.md) | Earlier 119.56 s native OpenVINS completion |
| [v117](gb10-sky-beta-course-v117/README.md) | Conservative profile, 123.84 s native OpenVINS completion |
| [cuVSLAM comparison](cuvslam-profile-comparison-v1/README.md) | Organizer-profile stereo-only and inertial trials failed before the first gate |

Each original report describes its own scene, reference and configuration.
"Current fastest" in an older report means at that report's date. Native results
use simulator source time and independently sampled contact/gate evaluation;
sampling cannot prove continuous collision freedom. FlyX lab timing and policy
proxy timing are different quantities and must not be merged with these results.

Recorded video paths in copied reports are not hosted links. Preserve videos,
ULogs, frame timestamps, failures and source snapshots separately when transferring
the repository. The private repository is intentionally source-sized.
