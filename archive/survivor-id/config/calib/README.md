# Per-robot numbers

Any `*.yaml` file here is merged on top of `config/survivor.yaml` when the configuration is
loaded, so measured values (the body-scale constants from stage 3 of `DEPLOY.md`, tuned
thresholds, the pose model's real speed) live here and `survivor.yaml` keeps the defaults.

Example, `config/calib/measured.yaml`:

```yaml
person:
  torso_m: 0.51
budget:
  inferences_per_s: 22
```

Camera lens models and mounting poses are not here: they come from
`projectile-dodging/config/` (and its `calib/`), which both components share.
