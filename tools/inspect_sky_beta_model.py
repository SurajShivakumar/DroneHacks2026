"""Passive rigid-body checks and a still of Sky Beta CAD; no thrust or flight."""
import argparse
from hashlib import sha256
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    contract_path = args.model/'airframe-contract.json'
    contract = json.loads(contract_path.read_text())
    model = mujoco.MjModel.from_xml_path(str((args.model/'course-sky-beta.xml').resolve()))
    data = mujoco.MjData(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'sky_beta')
    mujoco.mj_forward(model, data)
    initial_z = float(data.xpos[body, 2])
    initial_contacts = data.ncon
    for _ in range(24):
        mujoco.mj_step(model, data)
    actual_drop = initial_z-float(data.qpos[2])
    analytic_drop = .5*abs(model.opt.gravity[2])*data.time**2
    if initial_contacts or data.ncon or abs(actual_drop-analytic_drop) > .003:
        raise RuntimeError('passive early freefall disagrees with gravity or has contacts')
    for _ in range(456):
        mujoco.mj_step(model, data)
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or data.ncon < 1:
        raise RuntimeError('passive drop did not produce a finite contacting rigid body')
    report = {'kind': 'passive_sky_beta_rigid_body_check_no_motor_or_flight',
              'model_contract_sha256': sha256(contract_path.read_bytes()).hexdigest(),
              'mass_kg': float(model.body_mass[body]), 'principal_inertia_kg_m2': model.body_inertia[body].tolist(),
              'freefall_0p1s_drop_m': actual_drop, 'analytic_drop_m': float(analytic_drop),
              'final_time_s': float(data.time), 'final_com_z_m': float(data.qpos[2]),
              'final_contact_count': data.ncon, 'actuator_count': model.nu,
              'flight_model_ready': False, 'hardware_qualified': False}
    (args.output/'result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    # Display aid only: hide arena solids so the close-up is unobstructed.
    for i in range(model.ngeom):
        if model.geom_bodyid[i] == 0:
            model.geom_rgba[i, 3] = 0
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = data.xpos[body]
    camera.distance, camera.elevation, camera.azimuth = .65, -35, 130
    with mujoco.Renderer(model, width=1280, height=720) as renderer:
        renderer.update_scene(data, camera)
        image = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 1280, 62), fill='black')
        draw.text((12, 10), 'SKY BETA | SUPPLIED CAD + STL | millimetres -> metres | forward CAD -Y -> body +X', fill='white')
        draw.text((12, 34), f'CAD mass {report["mass_kg"]:.4f} kg; propellers/wiring/rangefinder omitted. Actuator model pending. No flight.', fill='white')
        image.save(args.output/'sky-beta.png')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
