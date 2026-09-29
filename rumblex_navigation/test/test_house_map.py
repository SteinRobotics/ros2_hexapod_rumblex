"""Check the metric footprint and occupancy classes of the house map."""

from collections import deque
from pathlib import Path
import unittest

import yaml


class HouseMapTest(unittest.TestCase):
    def test_house_contour(self):
        maps = Path(__file__).resolve().parents[1] / 'maps'
        metadata = yaml.safe_load((maps / 'house_contour.yaml').read_text())
        with (maps / metadata['image']).open('rb') as image:
            self.assertEqual(image.readline().strip(), b'P5')
            line = image.readline()
            while line.startswith(b'#'):
                line = image.readline()
            width, height = map(int, line.split())
            self.assertEqual(image.readline().strip(), b'255')
            pixels = image.read()
        self.assertEqual(len(pixels), width * height)
        self.assertEqual(metadata['origin'], [-4.95, -5.15, 0.0])
        resolution = metadata['resolution']
        counts = {value: pixels.count(value) for value in (0, 205, 254)}
        self.assertEqual(sum(counts.values()), len(pixels))
        self.assertGreater(counts[0], 445 * 465 - 431 * 451)
        occupied = [(i % width, i // width) for i, value in enumerate(pixels) if value == 0]
        xs, ys = zip(*occupied)
        self.assertAlmostEqual((max(xs) - min(xs) + 1) * resolution, 8.90)
        self.assertAlmostEqual((max(ys) - min(ys) + 1) * resolution, 9.30)
        for y in range(height):
            for x in range(width):
                value = pixels[y * width + x]
                if 32 <= x < width - 32 and 32 <= y < height - 32:
                    self.assertIn(value, (0, 254))
                elif 25 <= x < width - 25 and 25 <= y < height - 25:
                    self.assertEqual(value, 0)
                else:
                    self.assertEqual(value, 205)

        def cell(x, y):
            # Coordinates measured from the house's lower-left exterior corner.
            return (int((x + 0.50) / resolution),
                    height - 1 - int((y + 0.50) / resolution))

        def value_at(x, y):
            col, row = cell(x, y)
            return pixels[row * width + col]

        # Stair flights and landing must all be blocked, not striped/free treads.
        for x in (5.5, 6.5, 7.5, 8.5):
            for y in (3.8, 4.5, 5.5):
                self.assertEqual(value_at(x, y), 0)
        for x, y in ((2, 4.96), (4.3, 8), (5.24, 8), (7, 5.92), (7, 3.6)):
            self.assertEqual(value_at(x, y), 0)

        # Every room must remain connected to the hall through an open doorway.
        start = cell(4.8, 6.4)
        visited = {start}
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if (0 <= nx < width and 0 <= ny < height
                        and (nx, ny) not in visited and pixels[ny * width + nx] == 254):
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        for room in ((2, 8), (4.7, 8), (7, 8), (2, 2), (7, 2)):
            self.assertIn(cell(*room), visited)
        self.assertEqual(len(visited), counts[254])


if __name__ == '__main__':
    unittest.main()
