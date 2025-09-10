# AntobotCamera

## Dual recording

To make use of dual recording, the `platform_config.yaml` must have the `dual: true` entry at the level shown below. If the entry is absent or false, the camera will record from a single camera. 

```yaml 
camera:
  RP:
    mode: "recording"
    location: "left"
    dual: true
```