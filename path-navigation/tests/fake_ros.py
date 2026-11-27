"""A stand-in for the ROS 2 Python API, just enough to run robot/nav_node.py off the robot.

install() puts fake rclpy, message and service modules into sys.modules; a test then imports
the real node file. Topics are delivered synchronously. Timers do not run by themselves: a
test calls the node's tick.
"""
from __future__ import annotations

import sys
import types


class Bus:
    def __init__(self):
        self.subs, self.sent, self.calls, self.publishers = {}, {}, [], set()

    def publish(self, topic, msg):
        self.sent.setdefault(topic, []).append(msg)
        for cb in list(self.subs.get(topic, [])):
            cb(msg)


BUS = Bus()


class Msg:
    FIELDS: dict = {}

    def __init__(self, **kw):
        for k, v in self.FIELDS.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)


def msg(_type_name, **fields):
    return type(_type_name, (Msg,), {"FIELDS": fields})


Vector3 = msg("Vector3", x=0.0, y=0.0, z=0.0)
Quaternion = msg("Quaternion", x=0.0, y=0.0, z=0.0, w=1.0)
String = msg("String", data="")
Float32 = msg("Float32", data=0.0)
Float64 = msg("Float64", data=0.0)
Empty = msg("Empty")
Twist = msg("Twist", linear=Vector3, angular=Vector3)
Imu = msg("Imu", orientation=Quaternion, angular_velocity=Vector3)
Vector3Stamped = msg("Vector3Stamped", vector=Vector3)
Controller = msg("Controller", name="", type="", state="")


class SetBool:
    Request = msg("SetBoolRequest", data=False)
    Response = msg("SetBoolResponse", success=True, message="")


class ListControllers:
    Request = msg("ListRequest")
    Response = msg("ListResponse", controller=list)


class Future:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, cb):
        cb(self)


class Client:
    def __init__(self, bus, name):
        self.bus, self.name = bus, name

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.bus.calls.append((self.name, request))
        if self.name.endswith("list_controllers"):
            return Future(ListControllers.Response(controller=[Controller(name="gemini_walk", type="neural_controller/NeuralController", state="active")]))
        return Future(SetBool.Response())


class Publisher:
    def __init__(self, bus, topic):
        self.bus, self.topic = bus, topic

    def publish(self, m):
        self.bus.publish(self.topic, m)

    def get_subscription_count(self):
        return len(self.bus.subs.get(self.topic, []))


class Logger:
    def info(self, *_): pass
    warning = error = debug = info


class Node:
    def __init__(self, name):
        self.name, self.bus, self.params, self.timers = name, BUS, {}, []

    def declare_parameter(self, name, default=None):
        self.params.setdefault(name, default)
        return types.SimpleNamespace(value=self.params[name])

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.params.get(name))

    def create_publisher(self, _type, topic, _qos):
        return Publisher(self.bus, topic)

    def create_subscription(self, _type, topic, cb, _qos):
        self.bus.subs.setdefault(topic, []).append(cb)

    def create_client(self, _type, name):
        return Client(self.bus, name)

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def count_publishers(self, topic):
        return 1 if topic in self.bus.publishers else 0

    def get_logger(self):
        return Logger()

    def get_clock(self):
        import time
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(time.time() * 1e9)))

    def destroy_node(self):
        pass


def install(params: dict | None = None) -> Bus:
    """Fresh bus, fake modules in place. `params` pre-sets node parameters (as -p would)."""
    global BUS
    BUS = Bus()
    preset = dict(params or {})

    class PresetNode(Node):
        def __init__(self, name):
            super().__init__(name)
            self.params.update(preset)

    def module(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    module("rclpy", init=lambda *a, **k: None, shutdown=lambda: None, ok=lambda: True, spin=lambda n: None)
    module("rclpy.node", Node=PresetNode)
    module("std_msgs"); module("std_msgs.msg", String=String, Empty=Empty, Float32=Float32, Float64=Float64)
    module("geometry_msgs"); module("geometry_msgs.msg", Twist=Twist, Vector3Stamped=Vector3Stamped)
    module("sensor_msgs"); module("sensor_msgs.msg", Imu=Imu)
    module("std_srvs"); module("std_srvs.srv", SetBool=SetBool)
    module("controller_manager_msgs"); module("controller_manager_msgs.srv", ListControllers=ListControllers)
    return BUS
