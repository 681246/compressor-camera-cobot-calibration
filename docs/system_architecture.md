# System Architecture

## Industrial Context

This calibration program is designed for a compressor inspection system involving:

- gantry movement
- UR cobot positioning
- camera-guided inspection reference
- compressor placement offset
- trained inspection checkpoints

## Logical Architecture

```text
Trained Data
    ↓
Common Reference Pose
    ↓
Target / Checkpoint Pose
    ↓
Transformation Matrix Formation
    ↓
Pose Difference Calculation
    ↓
Compressor Offset Application
    ↓
Corrected Gantry + Cobot Pose
    ↓
Repeatable Camera / Inspection Position
```

## Software Modules

| Module | Responsibility |
|---|---|
| `Formation` | Matrix generation and UR pose conversion |
| `Calculate` | Full calibration pipeline |
| `Calculate2` | Simple offset application helper |
| Samples | Demonstrate input/output structure |
| Docs | Explain industrial and robotics context |
| Tests | Provide basic regression-test direction |

## Hardware Dependency

This portfolio version does not connect directly to hardware. It demonstrates the mathematical and software logic that can be integrated with:

- PLC motion command layer
- UR cobot RTDE/secondary interface
- camera capture pipeline
- inspection sequence controller
- database configuration system
