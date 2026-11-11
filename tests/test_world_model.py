from pathlib import Path
import unittest

import dronebench.world_model as world_model
from dronebench.depth_mapping import DepthMap
from dronebench.experiments import STRATEGIES
from dronebench.world_model import (
    Goal, GoalSequence, MotionLimits, MotionModel, Pose, WorldModel,
)


RESOLUTION = 0.5


def observed_map(shape=(16, 16, 4)):
    mapping = DepthMap(shape, RESOLUTION)
    mapping.cells[:] = 0
    return mapping


def paint_wall(mapping, column, open_rows):
    open_rows = set(open_rows)
    for layer in range(mapping.shape[2]):
        for row in range(mapping.shape[1]):
            mapping.cells[column, row, layer] = 0 if row in open_rows else 100


def cell_center(column, row, z=0.25):
    return Pose((column + 0.5) * RESOLUTION, (row + 0.5) * RESOLUTION, z, 0.0, 0.0)


def world(mapping):
    motion = MotionModel(MotionLimits(max_speed=0.6, max_accel=2.0, max_yaw_rate=3.0))
    return WorldModel(mapping, motion, dt=0.1)


class MotionTests(unittest.TestCase):
    def test_prediction_respects_acceleration_and_yaw_limits(self):
        motion = MotionModel(MotionLimits(max_speed=5, max_accel=2, max_yaw_rate=1))
        pose = Pose(0, 0, 1, 0, 0)
        moved = motion.predict(pose, 0, 4, 0.5)
        self.assertAlmostEqual(moved.speed, 1)
        self.assertAlmostEqual(moved.x, 0.5)
        self.assertAlmostEqual(moved.y, 0)
        turned = motion.predict(pose, 10, 0, 0.5)
        self.assertAlmostEqual(turned.yaw, 0.5)
        self.assertEqual((turned.x, turned.y, turned.speed), (0, 0, 0))

    def test_open_loop_imagination_matches_stepwise_prediction(self):
        mapping = observed_map()
        model = world(mapping)
        start = Pose(1, 1, 0.25, 0, 0)
        controls = ((0, 0.6), (1, 0.6))
        rollout = model.imagine(start, controls)
        pose = start
        for command in controls:
            pose = model.motion.predict(pose, *command, model.dt)
        self.assertEqual(rollout[-1], pose)
        self.assertAlmostEqual(pose.speed, 0.4)
        self.assertAlmostEqual(pose.yaw, 0.1)


class GeneralityTests(unittest.TestCase):
    def setUp(self):
        self.bands = [(0.0, 0.5)]
        self.goal = cell_center(13, 8)

    def _plan(self, mapping):
        return world(mapping).plan(
            cell_center(2, 8), (self.goal.x, self.goal.y), self.bands,
            radius=0.05, half_height=0.2)

    def test_same_planner_crosses_whichever_gap_the_map_shows(self):
        south = observed_map()
        paint_wall(south, 8, range(1, 6))
        north = observed_map()
        paint_wall(north, 8, range(11, 16))
        south_plan = self._plan(south)
        north_plan = self._plan(north)
        self.assertEqual(south_plan.kind, "goal")
        self.assertEqual(north_plan.kind, "goal")
        self.assertTrue(any(y < 3.5 for _x, y in south_plan.path))
        self.assertTrue(any(y > 5.0 for _x, y in north_plan.path))
        self.assertTrue(all(world(south).occupancy.clear(pose, 0.05, 0.2) for pose in south_plan.poses))
        self.assertTrue(all(world(north).occupancy.clear(pose, 0.05, 0.2) for pose in north_plan.poses))
        self.assertFalse(all(world(north).occupancy.clear(pose, 0.05, 0.2) for pose in south_plan.poses))

    def test_blocked_known_map_does_not_invent_a_route_or_a_border_frontier(self):
        mapping = observed_map()
        paint_wall(mapping, 8, ())
        self.assertIsNone(self._plan(mapping))

    def test_unknown_curtain_stops_at_a_frontier_until_the_map_changes(self):
        mapping = observed_map()
        mapping.cells[8:, :, :] = -1
        model = world(mapping)
        start = cell_center(2, 8)
        goal_xy = (self.goal.x, self.goal.y)
        first = model.plan(start, goal_xy, self.bands, radius=0.05, half_height=0.2)
        self.assertEqual(first.kind, "frontier")
        self.assertTrue(all(x < 8 * RESOLUTION for x, _y in first.path))
        self.assertTrue(all(model.occupancy.clear(pose, 0.05, 0.2) for pose in first.poses))
        mapping.cells[8:, :, :] = 0
        second = model.plan(start, goal_xy, self.bands, radius=0.05, half_height=0.2)
        self.assertEqual(second.kind, "goal")
        self.assertAlmostEqual(second.path[-1][0], self.goal.x)
        self.assertGreater(second.predicted_seconds, 0)

    def test_altitude_comes_from_the_free_band_rather_than_a_named_opening(self):
        mapping = observed_map()
        for row in range(mapping.shape[1]):
            mapping.cells[8, row, 1] = 100
        model = world(mapping)
        bands = [(0.5, 1.0), (1.5, 2.0)]
        plan = model.plan(cell_center(2, 8, z=0.75), (self.goal.x, self.goal.y), bands,
                          radius=0.05, half_height=0.2)
        self.assertEqual(plan.kind, "goal")
        self.assertEqual(plan.band, (1.5, 2.0))
        self.assertTrue(all(abs(pose.z - 1.75) < 1e-9 for pose in plan.poses))
        low = model.plan(cell_center(2, 8, z=0.75), (self.goal.x, self.goal.y), [(0.5, 1.0)],
                         radius=0.05, half_height=0.2)
        self.assertIsNone(low)

    def test_goal_list_is_data_and_cannot_skip_ahead(self):
        goals = GoalSequence((
            Goal("alpha", (1.0, 1.0, 0.25), 0.2),
            Goal("beta", (4.0, 1.0, 0.25), 0.2),
        ))
        self.assertEqual(goals.observe(Pose(4, 1, 0.25, 0, 0)), "alpha")
        self.assertEqual(goals.observe(Pose(1, 1, 0.25, 0, 0)), "beta")
        self.assertEqual(goals.observe(Pose(4, 1, 0.25, 0, 0)), "COMPLETE")
        self.assertEqual([goal.label for goal in goals.accepted], ["alpha", "beta"])

    def test_finer_grid_uses_the_same_planner(self):
        mapping = DepthMap((20, 8, 2), 0.1)
        mapping.cells[:] = 0
        motion = MotionModel(MotionLimits(max_speed=0.6, max_accel=2.0, max_yaw_rate=3.0))
        model = WorldModel(mapping, motion, dt=0.1)
        start = Pose(0.25, 0.45, 0.05, 0.0, 0.0)
        goal = (1.65, 0.45)
        plan = model.plan(start, goal, [(0.0, 0.1)], radius=0.02, half_height=0.04)
        self.assertEqual(plan.kind, "goal")
        self.assertAlmostEqual(plan.path[-1][0], goal[0])
        self.assertTrue(all(model.occupancy.clear(pose, 0.02, 0.04) for pose in plan.poses))

    def test_module_does_not_name_this_course(self):
        text = Path(world_model.__file__).read_text(encoding="utf-8")
        for name in ("gate_b", "maze_exit", "FIRST_CORRIDOR", "chicane", "window"):
            self.assertNotIn(name, text)
        self.assertEqual(STRATEGIES["world_model"]["status"], "offline_component_flight_pending")
        self.assertIn("goal_list", STRATEGIES["world_model"]["generality"])


class GoalPlannerTests(unittest.TestCase):
    def test_replanning_uses_only_the_active_goal(self):
        mapping = observed_map()
        paint_wall(mapping, 8, range(1, 6))
        near = cell_center(4, 8)
        far = cell_center(13, 8)
        goals = GoalSequence((
            Goal("near", (near.x, near.y, near.z), 0.3),
            Goal("far", (far.x, far.y, far.z), 0.3),
        ))
        model = world(mapping)
        first = model.plan_current(cell_center(2, 8), goals, [(0.0, 0.5)], radius=0.05, half_height=0.2)
        self.assertEqual(first.kind, "goal")
        self.assertLess(first.path[-1][0], 3.0)
        goals.observe(Pose(near.x, near.y, near.z, 0, 0))
        second = model.plan_current(near, goals, [(0.0, 0.5)], radius=0.05, half_height=0.2)
        self.assertEqual(goals.current.label, "far")
        self.assertEqual(second.kind, "goal")
        self.assertGreater(second.path[-1][0], 6.0)
        self.assertTrue(any(y < 3.5 for _x, y in second.path))
