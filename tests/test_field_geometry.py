import unittest

from ultimate_frisbee_simulator import FieldGeometry, FieldRegion


class FieldGeometryTests(unittest.TestCase):
    def test_regulation_defaults(self):
        field = FieldGeometry()

        self.assertEqual(field.length_m, 100.0)
        self.assertEqual(field.width_m, 37.0)
        self.assertEqual(field.end_zone_depth_m, 18.0)
        self.assertEqual(field.playing_area_length_m, 64.0)

    def test_custom_dimensions_derive_playing_area(self):
        field = FieldGeometry(length_m=120.0, width_m=50.0, end_zone_depth_m=20.0)

        self.assertEqual(field.playing_area_length_m, 80.0)
        self.assertEqual(field.classify_position(30.0, 25.0), FieldRegion.PLAYING_AREA)

    def test_classifies_both_end_zones_and_the_playing_area(self):
        field = FieldGeometry()

        self.assertEqual(field.classify_position(10.0, 10.0), FieldRegion.END_ZONE_A)
        self.assertEqual(field.classify_position(50.0, 10.0), FieldRegion.PLAYING_AREA)
        self.assertEqual(field.classify_position(90.0, 10.0), FieldRegion.END_ZONE_B)

    def test_goal_line_boundaries_belong_to_end_zones(self):
        field = FieldGeometry()

        self.assertEqual(field.classify_position(18.0, 10.0), FieldRegion.END_ZONE_A)
        self.assertEqual(field.classify_position(82.0, 10.0), FieldRegion.END_ZONE_B)

    def test_field_edges_are_included_and_positions_beyond_are_outside(self):
        field = FieldGeometry()

        self.assertEqual(field.classify_position(0.0, 0.0), FieldRegion.END_ZONE_A)
        self.assertEqual(field.classify_position(100.0, 37.0), FieldRegion.END_ZONE_B)
        self.assertEqual(field.classify_position(-0.01, 10.0), FieldRegion.OUTSIDE)
        self.assertEqual(field.classify_position(10.0, 37.01), FieldRegion.OUTSIDE)

    def test_rejects_non_positive_non_finite_and_non_numeric_dimensions(self):
        for value in (0.0, -1.0, float("nan"), float("inf"), True, "100"):
            with self.subTest(length=value):
                with self.assertRaises(ValueError):
                    FieldGeometry(length_m=value)

    def test_rejects_invalid_end_zone_depth(self):
        for depth in (0.0, -1.0, 50.0):
            with self.subTest(end_zone_depth=depth):
                with self.assertRaises(ValueError):
                    FieldGeometry(end_zone_depth_m=depth)

    def test_rejects_invalid_width(self):
        with self.assertRaises(ValueError):
            FieldGeometry(width_m=0.0)

    def test_rejects_non_finite_position(self):
        with self.assertRaises(ValueError):
            FieldGeometry().classify_position(float("nan"), 5.0)


if __name__ == "__main__":
    unittest.main()
