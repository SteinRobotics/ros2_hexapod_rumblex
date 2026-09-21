"""Headless geometry regressions. Run: python -m unittest test_cad_geometry."""

# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import unittest

from build123d import Circle, Pos, Rot

import robot_nox.assembly_body as assembly_body
import robot_nox.assembly_complete as assembly_complete
import robot_nox.assembly_leg as assembly_leg
import robot_nox.body_common as body_common
import robot_nox.body_layer_3 as body_layer_3
import common.bracket_inclined as bracket_inclined
import common.bracket_u_shape as bracket_u_shape
import robot_nox.foot_common as foot_common
import common.servo_simplified as servo_simplified


class GeometryTests(unittest.TestCase):
    def test_body_layers_seat_on_spacers(self):
        body = assembly_body.build_assembly()
        self.assertTrue(body.is_valid)
        parts = {child.label: child for child in body.children}
        layers = [parts[f"body_layer_{index}"] for index in range(4)]
        self.assertAlmostEqual(layers[0].bounding_box().min.Z, 0.0)
        for index, group in enumerate(("spacer_0_1", "spacer_1_2", "spacer_top")):
            for hole_index, location in enumerate(body_common.hole_locations):
                spacer = parts[f"{group}_{hole_index}"]
                box = spacer.bounding_box()
                self.assertAlmostEqual(box.min.Z, layers[index].bounding_box().max.Z)
                self.assertAlmostEqual(box.max.Z, layers[index + 1].bounding_box().min.Z)
                self.assertAlmostEqual(box.center().X, location.position.X)
                self.assertAlmostEqual(box.center().Y, location.position.Y)

    def test_top_plate_keeps_common_openings_and_spacer_clearance(self):
        surface = body_layer_3.build_surface()
        part = body_layer_3.build_model(surface)
        self.assertTrue(part.is_valid)
        self.assertEqual(len(part.solids()), 1)
        self.assertEqual(len(surface.faces()), 1)
        face = surface.faces()[0]
        self.assertEqual(len(face.outer_wire().edges()), 8)
        # One inner opening, 18 slots, and eight mounting holes.
        self.assertEqual(len(face.inner_wires()), 27)
        expected = body_common.base_plate.sketch & body_common.build_surface()
        self.assertLess((surface - expected).area, 1e-6)
        self.assertLess((expected - surface).area, 1e-6)
        for location in body_common.hole_locations:
            footprint = location * Circle(body_common.spacer_outer_radius)
            self.assertGreaterEqual(face.outer_wire().distance_to(footprint), 3.25 - 1e-6)
        self.assertAlmostEqual(part.volume, surface.area * body_common.THICKNESS)

    def assert_joint_mates(self, first, second):
        self.assertLess((first.location.position - second.location.position).length, 1e-6)
        first_axis = first.location.z_axis.direction
        second_axis = second.location.z_axis.direction
        self.assertAlmostEqual(abs(first_axis.dot(second_axis)), 1.0)

    def test_leg_joints_at_rotated_servo(self):
        servo = Pos(80, -45, 20) * Rot(25, 40, 70) * servo_simplified.build_model()
        original_location = servo.location
        leg = assembly_leg.attach_leg(servo, coxa_angle=25, femur_angle=-35, tibia_angle=70)
        coxa, femur, tibia = leg.children
        self.assertEqual(servo.location, original_location)
        self.assert_joint_mates(servo.joints["rotation"], coxa.joints["body_to_coxa_fixed"])
        self.assert_joint_mates(coxa.joints["coxa_to_femur_fixed"], femur.joints["femur_to_coxa_revolute"])
        self.assert_joint_mates(femur.joints["femur_to_tibia_fixed"], tibia.joints["tibia_to_femur_revolute"])
        self.assertTrue(leg.is_valid)

        # The femur is rolled lengthwise: its rear horn seats at the coxa
        # interface, while its length direction stays unchanged.
        femur_servo = femur.children[0]
        self.assertAlmostEqual(femur_servo.location.z_axis.direction.Z, 1.0)
        self.assertAlmostEqual(femur_servo.location.x_axis.direction.X, -1.0)
        rear_horn = Pos(
            0,
            servo_simplified.BODY_Y + servo_simplified.HORN_DISTANCE_TO_BODY + servo_simplified.HORN_BACK_Y,
            servo_simplified.HORN_Z_CTR,
        )
        rear_horn_position = (femur.location * femur_servo.location * rear_horn).position
        self.assertLess(
            (rear_horn_position - coxa.joints["coxa_to_femur_fixed"].location.position).length, 1e-6
        )

        # Both foot plates must put their mounting holes on the servo axes.
        foot = tibia.children[1]
        servo_part = tibia.children[0]
        # Rolling the tibia preserves its length direction and swaps horn faces.
        self.assertAlmostEqual(servo_part.location.x_axis.direction.X, 1.0)
        self.assertAlmostEqual(servo_part.location.z_axis.direction.Z, -1.0)
        tibia_rear_horn = (tibia.location * servo_part.location * rear_horn).position
        self.assertLess(
            (tibia_rear_horn - femur.joints["femur_to_tibia_fixed"].location.position).length, 1e-6
        )
        for plate in (foot.children[2], foot.children[3]):
            for x, y, _ in servo_simplified.SERVO_BRACKET_HOLES:
                local_hole = (
                    servo_part.location.inverse() * foot.location * plate.location * Pos(x, y, 0)
                ).position
                self.assertAlmostEqual(abs(local_hole.X), servo_simplified.HOLE_X)
                self.assertLess(min(abs(local_hole.Z - z) for z in
                                    (servo_simplified.HOLE_Z_LOW, servo_simplified.HOLE_Z_HIGH)), 1e-6)

    def test_complete_robot_connections(self):
        robot = assembly_complete.build_assembly()
        body, yaw, head, *legs = robot.children
        servos = {child.label: child for child in body.children}
        self.assertEqual(len(legs), 6)
        self.assertTrue(robot.is_valid)
        self.assert_joint_mates(servos["servo_head"].joints["rotation"], yaw.joints["body_to_coxa_fixed"])
        self.assert_joint_mates(yaw.joints["coxa_to_femur_fixed"], head.joints["pitch"])
        servos["servo_head"].joints["rotation"].connect_to(yaw.joints["body_to_coxa_fixed"], angle=25)
        yaw.joints["coxa_to_femur_fixed"].connect_to(head.joints["pitch"], angle=35)
        self.assert_joint_mates(servos["servo_head"].joints["rotation"], yaw.joints["body_to_coxa_fixed"])
        self.assert_joint_mates(yaw.joints["coxa_to_femur_fixed"], head.joints["pitch"])
        for leg in legs:
            servo = servos[leg.label.replace("leg_", "servo_", 1)]
            self.assert_joint_mates(servo.joints["rotation"], leg.children[0].joints["body_to_coxa_fixed"])
            tibia = leg.children[2]
            foot = tibia.children[1]
            toe = foot.children[1]
            toe_center = (leg.location * tibia.location * foot.location * Pos(toe.center())).position
            self.assertLess(toe_center.Z, 0, "Display pose must put the feet below the body")

    def test_bracket_faces_fit_horn_span(self):
        horn_span = (servo_simplified.BODY_Y + servo_simplified.HORN_BACK_Y
                     + servo_simplified.HORN_FRONT_Y + 2 * servo_simplified.HORN_DISTANCE_TO_BODY)
        for module in (bracket_u_shape, bracket_inclined):
            self.assertAlmostEqual(module.PLATE_W - 2 * module.LEG_T, horn_span)
            bracket = module.build_bracket()
            self.assertTrue(bracket.is_valid)
            self.assertEqual(len(bracket.solids()), 1)

    def test_foot_outline_keeps_drawing_coordinates(self):
        box = foot_common.build_surface().bounding_box()
        self.assertAlmostEqual(box.min.X, min(x for x, _ in foot_common.FOOT_OUTLINE))
        self.assertAlmostEqual(box.max.X, max(x for x, _ in foot_common.FOOT_OUTLINE))


if __name__ == "__main__":
    unittest.main()
