#!/usr/bin/env python
#
# Copyright (c) 2021 Shivam Pandey
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
provide functions to control vehivle physics
"""

import carla

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.sensor import Sensor

from carla_msgs.msg import CarlaEgoVehicleControlWheel, \
    CarlaEgoVehicleControlGear, CarlaEgoVehicleControlPhysics


class PhysicsControl(PseudoActor):
    """
    provides functions to control vehicle physics
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identifying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(PhysicsControl, self).__init__(uid=uid,
                                             name=name,
                                             parent=parent,
                                             node=node)

        self.phycis_subscriber = self.node.new_subscription(CarlaEgoVehicleControlPhysics,
                                                            self.get_topic_prefix() + "/set_physics",
                                                            self.on_physics_control,
                                                            qos_profile=10)
        self.vehicle_physics_control = None

    def destroy(self):
        """
        Function (override) to destroy this object.

        Terminate ROS subscriptions
        Finally forward call to super class.

        :return:
        """
        self.node.destroy_subscription(self.phycis_subscriber)
        super(PhysicsControl, self).destroy()

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint indentifier for pseudo actor
        :return: name
        """
        return "actor.pseudo.physics"

    def on_physics_control(self, phy_ctrl):
        if self.parent and self.parent.carla_actor.is_alive:
            # Create Physics Control Object for instance
            self.vehicle_physics_control = self.parent.carla_actor.get_physics_control()

            for wheel in phy_ctrl.wheels: 
                wheel_ctrl = carla.WheelPhysicsControl(
                    tire_friction = wheel.tire_friction,
                    damping_rate = wheel.damping_rate,
                    max_steer_angle = wheel.max_steer_angle,
                    radius = wheel.radius,
                    max_brake_torque = wheel.max_brake_torque,
                    long_stiff_value = wheel.long_stiff_value,
                    lat_stiff_max_load = wheel.lat_stiff_max_load,
                    lat_stiff_value = wheel.lat_stiff_value
                )

                self.vehicle_physics_control.wheels[wheel.wheel_id] = wheel_ctrl

            # update torque curve if provided with correct input size >= 2 points
            if len(phy_ctrl.torque_curve) >= 2:
                self.vehicle_physics_control.torque_curve = []
                for trq_pt in phy_ctrl.torque_curve:
                    # Construct required vector points and update torque curve
                    self.vehicle_physics_control.torque_curve.append(carla.Vector2D(trq_pt.x, trq_pt.y))
          
            # update max_rpm if not 0(default)
            if phy_ctrl.max_rpm != 0:
                self.vehicle_physics_control.max_rpm = phy_ctrl.max_rpm

            # update moi
            if phy_ctrl.moi != 0:
                self.vehicle_physics_control.moi = phy_ctrl.moi

            # update damping_rate_full_throttle
            if phy_ctrl.damping_rate_full_throttle != 0:
                self.vehicle_physics_control.damping_rate_full_throttle = \
                    phy_ctrl.damping_rate_full_throttle

            # update damping_rate_zero_throttle_clutch_engaged
            if phy_ctrl.damping_rate_zero_throttle_clutch_engaged != 0:
                self.vehicle_physics_control.damping_rate_zero_throttle_clutch_engaged = \
                    phy_ctrl.damping_rate_zero_throttle_clutch_engaged
 
            # update damping_rate_zero_throttle_clutch_disengaged
            if phy_ctrl.damping_rate_zero_throttle_clutch_disengaged != 0:
                self.vehicle_physics_control.damping_rate_zero_throttle_clutch_disengaged = \
                    phy_ctrl.damping_rate_zero_throttle_clutch_disengaged

            # update use_gear_autobox
            self.vehicle_physics_control.use_gear_autobox = \
                phy_ctrl.use_gear_autobox

            # update clutch_strength
            if phy_ctrl.clutch_strength != 0:
                self.vehicle_physics_control.clutch_strength = \
                    phy_ctrl.clutch_strength

            # update final_ratio
            if phy_ctrl.final_ratio != 0:
                self.vehicle_physics_control.final_ratio = \
                    phy_ctrl.final_ratio

            # update forward_gears
            if len(phy_ctrl.forward_gears) != 0:
                self.vehicle_physics_control.forward_gears = []
                for gear in phy_ctrl.forward_gears:
                    self.vehicle_physics_control.forward_gears.append(
                        carla.GearPhysicsControl(ratio = gear.ratio, 
                                                 down_ratio = gear.down_ratio, 
                                                 up_ratio = gear.up_ratio
                                                 )
                    )
                    
            # update mass
            if phy_ctrl.mass != 0:
                self.vehicle_physics_control.mass = phy_ctrl.mass

            # update drag_coefficient
            if phy_ctrl.drag_coefficient != 0:
                self.vehicle_physics_control.drag_coefficient = \
                    phy_ctrl.drag_coefficient

            # update center_of_mass
            self.vehicle_physics_control.center_of_mass = \
                carla.Vector3D(
                    x = phy_ctrl.center_of_mass.x,
                    y = phy_ctrl.center_of_mass.y,
                    z = phy_ctrl.center_of_mass.z
                )

            # update steering_curve
            if len(phy_ctrl.steering_curve) >= 2:
                self.vehicle_physics_control.steering_curve = []
                for steer_pt in phy_ctrl.steering_curve:
                    # Construct required vector points and update torque curve
                    self.vehicle_physics_control.steering_curve.append(carla.Vector2D(steer_pt.x, steer_pt.y))

            # update use_sweep_wheel_collision
            self.vehicle_physics_control.use_sweep_wheel_collision = \
                phy_ctrl.use_sweep_wheel_collision

            # Apply Changes to carla_actor
            self.parent.carla_actor.apply_physics_control(self.vehicle_physics_control)
