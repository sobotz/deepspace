# Preferences

What all this spaghetti code means, and how you can make it work better through
the FRC driver station.

## The Drive Subsystem

| Field Name   | Type    | Explanation                                                                                 |
|--------------|---------|---------------------------------------------------------------------------------------------|
| RhinoEnabled | boolean | Whether or not rhino drive should be preferred over the legacy one-controller drive system. |
| JoystickInputAmplificationFactor | double  | The amount to amplify the joystick input by |

## The Intake Subsystem

| Field Name       | Type   | Explanation                                                |
|------------------|--------|------------------------------------------------------------|
| WristMaxPosition | double | The maximum position that the wrist can be moved to.       |
| WristIPosition   | double | The amount the wrist should be moved on each button press. |
