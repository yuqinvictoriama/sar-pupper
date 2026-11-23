"""Projectile dodging for Pupper: pure-Python core (no ROS).

    config          load config/dodge.yaml (+ config/calib/*.yaml overrides)
    camera_models   lens models and camera mounting poses
    detect          colour segmentation -> blobs
    measure         blob -> ray, angular size, range
    track           3D ballistic Kalman tracker
    threat          will the ball hit, when, from where
    escape          which way to move, how fast, for how long
    core            the state machine tying it together (DodgeCore)
    sim             throw simulator used by the tests
"""
