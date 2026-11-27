"""Just enough stand-in ROS 2 for the three nodes to be built and driven in a test.

    import fake_ros; fake_ros.install()      # before importing a node

Publishers remember what was published, subscriptions and services keep their callbacks so a
test can call them, timers never fire by themselves. Nothing here talks to anything.
"""
from __future__ import annotations

import sys
import types


class Msg:
    """A message whose fields appear when touched: msg.linear.x = 1.0 just works."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        value = Msg()
        self.__dict__[name] = value
        return value


def _msg(name: str, **defaults):
    def init(self, **kw):
        Msg.__init__(self, **{**{k: (v() if callable(v) else v) for k, v in defaults.items()}, **kw})
    return type(name, (Msg,), {"__init__": init})


def _vec():
    return Msg(x=0.0, y=0.0, z=0.0)


Twist = _msg("Twist", linear=_vec, angular=_vec)
String = _msg("String", data="")
CompressedImage = _msg("CompressedImage", header=lambda: Msg(stamp=Msg(sec=0, nanosec=0), frame_id=""), format="", data=b"")
Imu = _msg("Imu", header=lambda: Msg(stamp=Msg(sec=0, nanosec=0)), orientation=lambda: Msg(w=1.0, x=0.0, y=0.0, z=0.0),
           angular_velocity=_vec, linear_acceleration=_vec)
PoseStamped = _msg("PoseStamped", header=lambda: Msg(stamp=Msg(sec=0, nanosec=0), frame_id=""),
                   pose=lambda: Msg(position=_vec(), orientation=Msg(w=1.0, x=0.0, y=0.0, z=0.0)))


class _Srv:
    Request = _msg("Request", data=False)
    Response = _msg("Response", success=False, message="")


class Trigger(_Srv):
    pass


class SetBool(_Srv):
    pass


class ListControllers(_Srv):
    pass


class Publisher:
    def __init__(self, topic):
        self.topic, self.sent, self.listeners = topic, [], 1

    def publish(self, msg):
        self.sent.append(msg)

    def get_subscription_count(self):
        return self.listeners


class Future:
    def __init__(self, result):
        self._result = result

    def add_done_callback(self, cb):
        cb(self)

    def result(self):
        return self._result


class Client:
    def __init__(self, name):
        self.name, self.ready, self.answer = name, True, Msg(controller=[])

    def service_is_ready(self):
        return self.ready

    def call_async(self, _request):
        return Future(self.answer)


class Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, text):
        self.lines.append((level, str(text)))

    def info(self, text, **_kw):
        self._log("info", text)

    def warning(self, text, **_kw):
        self._log("warning", text)

    def error(self, text, **_kw):
        self._log("error", text)


class Node:
    params_for_next: dict = {}            # what `--ros-args -p name:=value` would have set

    def __init__(self, name):
        self.name = name
        self._params = dict(Node.params_for_next)
        Node.params_for_next = {}
        self.pubs, self.subs, self.services, self.clients, self.timers = {}, {}, {}, {}, []
        self._logger = Logger()

    def declare_parameter(self, name, default):
        self._params.setdefault(name, default)

    def get_parameter(self, name):
        return Msg(value=self._params[name])

    def create_publisher(self, _type, topic, _qos):
        self.pubs[topic] = Publisher(topic)
        return self.pubs[topic]

    def create_subscription(self, _type, topic, callback, _qos):
        self.subs[topic] = callback

    def create_service(self, _type, name, callback):
        self.services[name] = callback

    def create_client(self, _type, name):
        self.clients[name] = Client(name)
        return self.clients[name]

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass

    def call(self, service: str, **request):
        """Call one of this node's services the way a client would."""
        return self.services[service](Msg(**request), _Srv.Response())


def install() -> None:
    def module(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    module("rclpy", init=lambda *a, **k: None, ok=lambda: True, shutdown=lambda: None, spin=lambda node: None)
    module("rclpy.node", Node=Node)
    module("rclpy.executors", ExternalShutdownException=type("ExternalShutdownException", (Exception,), {}))
    for pkg, names in (("geometry_msgs", {"Twist": Twist, "PoseStamped": PoseStamped}),
                       ("sensor_msgs", {"CompressedImage": CompressedImage, "Imu": Imu}),
                       ("std_msgs", {"String": String})):
        module(pkg)
        module(pkg + ".msg", **names)
    module("std_srvs")
    module("std_srvs.srv", Trigger=Trigger, SetBool=SetBool)
    module("controller_manager_msgs")
    module("controller_manager_msgs.srv", ListControllers=ListControllers)
