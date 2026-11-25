"""Preview the captured CAD pose; solve loop closures when Joint sliders change.

Use --simulate to launch MuJoCo's normal physics viewer instead.
"""
import argparse
import copy
from pathlib import Path
import time

import mujoco
import numpy as np

MODEL_FILE = 'xiaochuanleg_fixed.xml'
INITIAL_JOINT_POSITIONS = {
}


class LoopProjector:
    """Kinematic editing only: hold edited coordinates and solve the remaining DOFs.

    The separate model skips contacts during IK without changing exported physics.
    Incremental targets keep the solution on the current assembly branch. Failed
    targets stop at the last closed configuration instead of tearing the loop.
    """

    tolerance = 1e-8

    def __init__(self, model):
        self.model = copy.copy(model)
        self.model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.data = mujoco.MjData(self.model)
        self.scalar_joints = [j for j in range(model.njnt) if model.jnt_type[j] in
                              (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)]

    def _clamp(self, qpos):
        for j in self.scalar_joints:
            if self.model.jnt_limited[j]:
                q = self.model.jnt_qposadr[j]
                qpos[q] = np.clip(qpos[q], *self.model.jnt_range[j])

    def _constraints(self, qpos):
        self.data.qpos[:] = qpos
        mujoco.mj_fwdPosition(self.model, self.data)
        rows = self.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        residual = self.data.efc_pos[rows].copy()
        jacobian = self.data.efc_J.reshape(self.data.nefc, self.model.nv)[rows].copy()
        return residual, jacobian

    def error(self, qpos):
        residual, _ = self._constraints(qpos)
        return float(np.max(np.abs(residual), initial=0))

    def _solve(self, qpos, free):
        qpos = qpos.copy()
        for _ in range(60):
            residual, jacobian = self._constraints(qpos)
            if np.max(np.abs(residual), initial=0) <= self.tolerance:
                return qpos
            if not np.any(free):
                break
            # Least squares handles the redundant rows of paired hinge connects.
            step = np.zeros(self.model.nv)
            step[free] = np.linalg.lstsq(jacobian[:, free], -residual, rcond=1e-8)[0]
            if not np.isfinite(step).all():
                break
            step *= min(1.0, 0.15 / max(np.max(np.abs(step)), 1e-15))
            improved = False
            for scale in (1, 0.5, 0.25, 0.125, 0.0625, 0.03125):
                trial = qpos.copy()
                mujoco.mj_integratePos(self.model, trial, step, scale)
                self._clamp(trial)
                updated, _ = self._constraints(trial)
                if np.linalg.norm(updated) < np.linalg.norm(residual):
                    qpos = trial
                    improved = True
                    break
            if not improved:
                break
        return None

    def project(self, previous, requested, active=None):
        """Return (closed pose, reached target), respecting scalar joint limits."""
        self.data.eq_active[:] = self.model.eq_active0 if active is None else active
        target = requested.copy()
        if not np.isfinite(target).all():
            return previous.copy(), False
        self._clamp(target)
        if self.error(target) <= self.tolerance:
            return target, np.allclose(target, requested, atol=1e-9, rtol=0)
        delta = np.zeros(self.model.nv)
        mujoco.mj_differentiatePos(self.model, delta, 1, previous, target)
        edited = np.abs(delta) > 1e-9
        free = ~edited
        # Do not use a floating base to compensate for an internal joint edit.
        for j in range(self.model.njnt):
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                adr = self.model.jnt_dofadr[j]
                free[adr:adr + 6] = False
            elif self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL:
                adr = self.model.jnt_dofadr[j]
                if np.any(edited[adr:adr + 3]):
                    free[adr:adr + 3] = False
        closed = previous.copy()
        if self.error(closed) > self.tolerance:
            # For example, an equality was just re-enabled in the viewer.
            repaired = self._solve(target, free)
            return (previous.copy(), False) if repaired is None else (repaired, True)
        magnitude = float(np.max(np.abs(delta), initial=0))
        if magnitude <= 1e-9:
            return closed, np.allclose(target, requested, atol=1e-9, rtol=0)
        increment = min(1.0, 0.04 / magnitude)
        progress = 0.0
        for _ in range(400):
            amount = min(increment, 1.0 - progress)
            trial = closed.copy()
            mujoco.mj_integratePos(self.model, trial, delta, amount)
            solved = self._solve(trial, free)
            if solved is None:
                increment *= 0.5
                if increment * magnitude < 1e-6:
                    break
                continue
            closed = solved
            progress += amount
            if progress >= 1.0 - 1e-12:
                return closed, np.allclose(target, requested, atol=1e-9, rtol=0)
        return closed, False


def load_model(path):
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    for name, value in INITIAL_JOINT_POSITIONS.items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint < 0:
            print(f'warning: joint not found: {name}')
            continue
        data.qpos[model.jnt_qposadr[joint]] = value
    mujoco.mj_forward(model, data)
    return model, data


def preview(model, data):
    import mujoco.viewer
    projector = LoopProjector(model)
    initial = data.qpos.copy()
    previous = initial.copy()
    previous_active = data.eq_active.copy()
    reset = False

    def on_key(key):
        nonlocal reset
        if key == ord('R'):
            reset = True

    print('Pose preview: drag Joint sliders; R resets the captured pose. No physics steps are run.')
    with mujoco.viewer.launch_passive(model, data, key_callback=on_key, show_left_ui=False) as viewer:
        viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, 'Pose preview | R: reset', 'Drag Joint sliders'))
        while viewer.is_running():
            # Keep intermediate, unprojected slider poses out of the rendered frame.
            with viewer.lock():
                viewer.sync()
                if reset:
                    mujoco.mj_resetData(model, data)
                    data.qpos[:] = initial
                    previous = initial.copy()
                    reset = False
                    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, 'Pose preview | R: reset', ''))
                changed = (not np.array_equal(data.qpos, previous) or
                           not np.array_equal(data.eq_active, previous_active))
                if changed:
                    data.qpos[:], reached = projector.project(previous, data.qpos, data.eq_active)
                    # An incompatible re-enabled constraint returns to the captured pose.
                    if projector.error(data.qpos) > projector.tolerance:
                        data.qpos[:] = initial
                        reached = False
                    data.qvel[:] = 0
                    data.qacc_warmstart[:] = 0
                    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                      'Pose preview | R: reset',
                                      '' if reached else 'Stopped at the reachable limit'))
                mujoco.mj_forward(model, data)
                previous = data.qpos.copy()
                previous_active = data.eq_active.copy()
                viewer.sync()
            time.sleep(1 / 60)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model', nargs='?', type=Path, default=Path(__file__).with_name(MODEL_FILE))
    parser.add_argument('--simulate', action='store_true', help='Run normal MuJoCo dynamics instead of pose preview')
    args = parser.parse_args()
    model, data = load_model(args.model.resolve())
    if args.simulate:
        import mujoco.viewer
        mujoco.viewer.launch(model, data)
    else:
        preview(model, data)


if __name__ == '__main__':
    main()
