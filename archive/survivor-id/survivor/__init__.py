"""Survivor identification for Pupper: pure-Python core (no ROS).

    config       load config/survivor.yaml (+ calib overrides) and the shared camera calibration
    cameras      views handed to the pose model, and pixel -> direction in the levelled body frame
    pose_decode  YOLOv8-pose head tensors -> people with 17 keypoints
    person       one detected person: joint directions, body scale, range, position
    gesture      the both-arms-up rule and its time filter
    appearance   clothing-colour descriptor for telling people apart
    tracker      dead reckoning, the people tracker and the survivor pin
    scheduler    which camera gets the wide pass and which people get a zoom pass
    perception   the above tied together (SurvivorPerception)
    mission      the approach / arrival state machine (Mission)
"""
