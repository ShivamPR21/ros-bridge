#!/usr/bin/env python
#
# Copyright (c) 2021 BOSCH India
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


class ControlVehiclePhysics():

    def __init__(self,vehicle):
        self.vehicle = vehicle
        self.physics_control = vehicle.get_physics_control()

    def CreateControl(self):   
        # Create Wheels Physics Control
        front_left_wheel  = carla.WheelPhysicsControl(tire_friction=4.5, damping_rate=1.0, max_steer_angle=70.0, radius=30.0)
        front_right_wheel = carla.WheelPhysicsControl(tire_friction=2.5, damping_rate=1.5, max_steer_angle=70.0, radius=25.0)
        rear_left_wheel   = carla.WheelPhysicsControl(tire_friction=1.0, damping_rate=0.2, max_steer_angle=0.0,  radius=15.0)
        rear_right_wheel  = carla.WheelPhysicsControl(tire_friction=1.5, damping_rate=1.3, max_steer_angle=0.0,  radius=20.0)
        wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]

        self.physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
        self.physics_control.max_rpm = 10000
        self.physics_control.moi = 1.0
        self.physics_control.damping_rate_full_throttle = 0.0
        self.physics_control.use_gear_autobox = True
        self.physics_control.gear_switch_time = 0.5
        self.physics_control.clutch_strength = 10
        self.physics_control.mass = 10000
        self.physics_control.drag_coefficient = 0.25
        self.physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1), carla.Vector2D(x=300, y=1)]
        self.physics_control.wheels = wheels
        self.physics_control.use_sweep_wheel_collision = True

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

