from math import pi
import unittest

from dronebench.grid import OccupancySlice, find_path
from dronebench.path_smoothing import LineOfSight, path_length, shortcut_path


class ShortcutTests(unittest.TestCase):
    def test_open_grid_shortcut_and_checked_arbitrary_connectors(self):
        grid = OccupancySlice(20, 20, .1, (0,) * 400)
        path = find_path(grid, (.35, .35), (1.55, 1.55), .05)
        result = shortcut_path(grid, path, .05, start=(.32, .37), goal=(1.57, 1.52))
        self.assertEqual(result, [(.32, .37), (1.57, 1.52)])
        self.assertLess(path_length(result), path_length(path))

    def test_wall_detour_is_retained_and_every_segment_clear(self):
        cells = [0] * 900
        for row in range(8, 22):
            cells[row * 30 + 15] = 100
        grid = OccupancySlice(30, 30, .1, tuple(cells))
        path = find_path(grid, (.55, 1.55), (2.55, 1.55), .1)
        result = shortcut_path(grid, path, .1)
        self.assertGreater(len(result), 2)
        self.assertLessEqual(path_length(result), path_length(path))
        self.assertTrue(all(LineOfSight(grid, .1).clear(a, b) for a, b in zip(result, result[1:])))

    def test_unknown_wall_and_unsafe_connector_rejected(self):
        cells = tuple(-1 if col == 10 else 0 for row in range(20) for col in range(20))
        grid = OccupancySlice(20, 20, 1., cells)
        checker = LineOfSight(grid, 0.)
        self.assertFalse(checker.clear((4.5, 5.5), (15.5, 5.5)))
        with self.assertRaises(ValueError):
            shortcut_path(grid, [(4.5, 5.5), (5.5, 5.5)], 0., goal=(15.5, 5.5))

    def test_corner_touch_is_blocked_even_if_interiors_are_free(self):
        cells = [0] * 100
        cells[5 * 10 + 5] = 100
        checker = LineOfSight(OccupancySlice(10, 10, 1., tuple(cells)), 0.)
        self.assertFalse(checker.clear((2.5, 5.5), (5.5, 2.5)))
        self.assertTrue(checker.clear((2.5, 5.5), (5.5, 2.4)))

    def test_edge_touch_and_outer_boundary_rejected(self):
        cells = [0] * 100
        cells[5 * 10 + 5] = 100
        checker = LineOfSight(OccupancySlice(10, 10, 1., tuple(cells)), 0.)
        self.assertFalse(checker.clear((4., 4.5), (4., 6.5)))
        self.assertFalse(checker.clear((0., 2.), (0., 3.)))
        self.assertFalse(checker.clear((-1., 2.), (2., 3.)))

    def test_rotated_grid_is_equivalent(self):
        grid = OccupancySlice(10, 10, .1, (0,) * 100, 2., 3., pi / 2)
        a, b = grid.cell_to_world(2, 2), grid.cell_to_world(7, 7)
        self.assertEqual(shortcut_path(grid, [a, grid.cell_to_world(2, 7), b], .05), [a, b])

    def test_footprint_blocks_narrow_corridor(self):
        cells = tuple(100 if col in (4, 10) else 0 for row in range(20) for col in range(15))
        grid = OccupancySlice(15, 20, .1, cells)
        self.assertFalse(LineOfSight(grid, .3).clear((.75, .55), (.75, 1.55)))
