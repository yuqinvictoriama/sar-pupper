"""Path navigation: the demo mission, in plain Python.

No ROS and no audio here. robot/nav_node.py feeds these classes on the robot,
sim/course_sim.py feeds the same classes in simulation, and the tests run both
ways on a laptop.

    config    every number of the demo (config/demo.yaml)
    heading   angles that wrap, and turning several bearings into one fix
    reckon    where Pupper is along the line, from the commands the legs were sent
    steer     walking a line
    events    what the mission reports, and the line Pupper says for each
    mission   the state machine
    board     the latest word from every other component, and the snapshot Mission Control draws
"""
