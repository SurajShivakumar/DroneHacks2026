from math import pi
import unittest

from dronebench.grid import OccupancySlice, find_path


class ObservedGridTests(unittest.TestCase):
    def test_routes_around_wall_instead_of_crossing_it(self):
        cells = [0] * 400
        for row in range(5, 15):
            cells[row * 20 + 10] = 100
        grid = OccupancySlice(20, 20, .1, tuple(cells))
        path = find_path(grid, grid.cell_to_world(4, 10), grid.cell_to_world(16, 10), .05)
        self.assertIsNotNone(path)
        self.assertTrue(any(y < .5 or y > 1.5 for x, y in path))

    def test_unknown_barrier_is_not_a_shortcut(self):
        cells = tuple(-1 if col == 10 else 0 for row in range(20) for col in range(20))
        grid = OccupancySlice(20, 20, .1, cells)
        self.assertIsNone(find_path(grid, grid.cell_to_world(4, 10), grid.cell_to_world(16, 10), .05))

    def test_vehicle_cannot_fit_through_too_narrow_gap(self):
        cells = tuple(100 if col in (4, 10) else 0 for row in range(20) for col in range(15))
        grid = OccupancySlice(15, 20, .1, cells)
        self.assertIsNone(find_path(grid, grid.cell_to_world(7, 5), grid.cell_to_world(7, 15), .3))

    def test_rotated_ros_grid_transform(self):
        grid = OccupancySlice(10, 10, .1, (0,) * 100, origin_x=2, origin_y=3, origin_yaw=pi / 2)
        point = grid.cell_to_world(3, 4)
        self.assertAlmostEqual(point[0], 1.55)
        self.assertAlmostEqual(point[1], 3.35)
        self.assertEqual(grid.world_to_cell(*point), (3, 4))

    def test_unchecked_endpoint_connector_is_rejected(self):
        grid = OccupancySlice(10, 10, .1, (0,) * 100)
        with self.assertRaises(ValueError):
            find_path(grid, (.31, .31), (.65, .65), .05)
