# Calibration for this robot

The tools write small YAML files here; they are merged over `config/dodge.yaml`
when the config is loaded (same key layout, later file names win).

| File | Written by |
|---|---|
| `devices.yaml` | `tools/find_cameras.py --assign` |
| `exposure.yaml` | `tools/tune_exposure.py` |
| `image_circle.yaml` | `tools/fit_image_circle.py --write` |
| `<camera>_color.yaml` | `tools/calibrate_color.py` |
| `<camera>_timing.yaml` | `tools/measure_latency.py --write` |
| `<camera>_intrinsics.yaml` | `tools/calibrate_camera.py` |
| `<camera>_extrinsics.yaml` | `tools/calibrate_extrinsics.py` |
| `step_response.yaml` | `tools/step_response.py --write` |

You can add your own, for example `mounting.yaml` or `tuning.yaml`, to override
any number without editing `dodge.yaml`:

```yaml
cameras:
  elp_left:
    extrinsics: {roll_deg: 180}
    capture: {fps: 60}
escape:
  vx_forward: 1.5
```

Delete a file to go back to the default for what it sets.
