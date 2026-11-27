"""A stand-in for the ROS 2 Python API, just enough to run robot/*.py off the robot.

``install()`` puts fake ``rclpy``, message and service modules into sys.modules; a test then
imports the real node files. Topics are delivered synchronously, in the publisher's thread.
``ControllerManager`` plays the controller manager's switch/list services with the rule
that matters here: two controllers cannot hold the legs at once.
"""

from __future__ import annotations

import sys
import threading
import types


class Bus:
    def __init__(self):
        self.subs = {}          # topic -> [callback]
        self.sent = {}          # topic -> [msg]
        self.services = {}      # name -> handler(request) -> response
        self.lock = threading.RLock()

    def publish(self, topic, msg):
        self.sent.setdefault(topic, []).append(msg)
        for cb in list(self.subs.get(topic, [])):
            cb(msg)


BUS = Bus()


class _Msg:
    def __init__(self, **kw):
        for k, v in self._fields().items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)

    @classmethod
    def _fields(cls):
        return {}


def _msg(_type_name, **fields):
    return type(_type_name, (_Msg,), {"_fields": classmethod(lambda cls: fields)})


Vector3 = _msg("Vector3", x=0.0, y=0.0, z=0.0)
Quaternion = _msg("Quaternion", x=0.0, y=0.0, z=0.0, w=1.0)
Header = _msg("Header", stamp=None)
String = _msg("String", data="")
Bool = _msg("Bool", data=False)
Empty = _msg("Empty")
Float64MultiArray = _msg("Float64MultiArray", data=list)
Twist = _msg("Twist", linear=Vector3, angular=Vector3)
JointState = _msg("JointState", name=list, position=list, velocity=list)
Imu = _msg("Imu", orientation=Quaternion, angular_velocity=Vector3)
CompressedImage = _msg("CompressedImage", header=Header, format="", data=b"")
Controller = _msg("Controller", name="", type="", state="")


class _Srv:
    pass


class SwitchController(_Srv):
    class Request(_Msg):
        STRICT, BEST_EFFORT = 2, 1

        @classmethod
        def _fields(cls):
            return {"activate_controllers": list, "deactivate_controllers": list, "strictness": 1}
    Response = _msg("SwitchResponse", ok=False)


class ListControllers(_Srv):
    Request = _msg("ListRequest")
    Response = _msg("ListResponse", controller=list)


class Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result

    def add_done_callback(self, cb):
        cb(self)


class Client:
    def __init__(self, name):
        self.name = name

    def wait_for_service(self, timeout_sec=None):
        return self.name in BUS.services

    def service_is_ready(self):
        return self.name in BUS.services

    def call_async(self, request):
        return Future(BUS.services[self.name](request))


class Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        BUS.publish(self.topic, msg)

    def get_subscription_count(self):
        return len(BUS.subs.get(self.topic, []))


class Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, text):
        self.lines.append((level, text))

    def info(self, text):
        self._log("info", text)

    def warning(self, text):
        self._log("warning", text)

    def error(self, text):
        self._log("error", text)


class _Clock:
    class _Now:
        def to_msg(self):
            return None

    def now(self):
        return self._Now()


class Node:
    def __init__(self, name):
        self.name = name
        self.timers = []
        self._logger = Logger()

    def create_publisher(self, _type, topic, _qos):
        return Publisher(topic)

    def create_subscription(self, _type, topic, cb, _qos):
        BUS.subs.setdefault(topic, []).append(cb)

    def create_client(self, _type, name):
        return Client(name)

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


class ControllerManager:
    """switch_controller / list_controllers. ``leg_holders``: controllers that claim the leg
    command interfaces; only one of the two groups (walking, terrain) may be active."""

    def __init__(self, walking=("neural_controller_default",), terrain=("terrain_position_controller",
                 "terrain_kp_controller", "terrain_kd_controller"), wait=None):
        self.walking, self.terrain = list(walking), list(terrain)
        self.state = {**{n: "inactive" for n in self.terrain}, **{n: "inactive" for n in self.walking}}
        self.state[self.walking[0]] = "active"
        self.types = {**{n: "forward_command_controller/ForwardCommandController" for n in self.terrain},
                      **{n: "neural_controller/NeuralController" for n in self.walking}}
        self.history = []
        self.wait = wait            # callable run inside a switch (to model the time it takes)
        self.on_switch = None       # callback(activated, deactivated)
        self.fail_next = False
        BUS.services["/controller_manager/switch_controller"] = self.switch
        BUS.services["/controller_manager/list_controllers"] = self.list

    def list(self, _req):
        return ListControllers.Response(controller=[Controller(name=n, type=self.types[n], state=s)
                                                    for n, s in self.state.items()])

    def switch(self, req):
        if self.wait:
            self.wait()
        strict = req.strictness == SwitchController.Request.STRICT
        act = [n for n in req.activate_controllers]
        deact = [n for n in req.deactivate_controllers]
        if self.fail_next:
            self.fail_next = False
            return SwitchController.Response(ok=False)
        if strict:
            if any(n not in self.state or self.state[n] == "active" for n in act):
                return SwitchController.Response(ok=False)
            if any(n not in self.state or self.state[n] != "active" for n in deact):
                return SwitchController.Response(ok=False)
        act = [n for n in act if n in self.state and self.state[n] != "active"]
        deact = [n for n in deact if n in self.state and self.state[n] == "active"]
        after = dict(self.state)
        for n in deact:
            after[n] = "inactive"
        for n in act:
            after[n] = "active"
        walking_on = any(after[n] == "active" for n in self.walking)
        terrain_on = any(after[n] == "active" for n in self.terrain)
        if walking_on and terrain_on:          # both would claim the leg command interfaces
            return SwitchController.Response(ok=False)
        self.state = after
        self.history.append((tuple(act), tuple(deact)))
        if self.on_switch:
            self.on_switch(act, deact)
        return SwitchController.Response(ok=True)

    def active(self, group):
        return [n for n in group if self.state[n] == "active"]


class _Executor:
    def __init__(self, num_threads=1):
        pass

    def add_node(self, node):
        pass

    def spin(self):
        pass


def install():
    """Register the fake modules. Safe to call more than once; resets the bus."""
    BUS.subs.clear()
    BUS.sent.clear()
    BUS.services.clear()

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    rclpy = mod("rclpy", init=lambda args=None: None, shutdown=lambda: None, ok=lambda: True,
                spin=lambda node: None, spin_once=lambda node, timeout_sec=0.0: None)
    rclpy.node = mod("rclpy.node", Node=Node)
    rclpy.executors = mod("rclpy.executors", MultiThreadedExecutor=_Executor, SingleThreadedExecutor=_Executor)
    mod("std_msgs", msg=mod("std_msgs.msg", String=String, Bool=Bool, Empty=Empty, Float64MultiArray=Float64MultiArray))
    mod("sensor_msgs", msg=mod("sensor_msgs.msg", JointState=JointState, Imu=Imu, CompressedImage=CompressedImage))
    mod("geometry_msgs", msg=mod("geometry_msgs.msg", Twist=Twist))
    mod("controller_manager_msgs", srv=mod("controller_manager_msgs.srv", SwitchController=SwitchController,
                                           ListControllers=ListControllers))
    return BUS
