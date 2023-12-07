# ProjectTemplate

This repository contains a basic ClearCore project that can be used as a template for new application development.
Place this repository rooted in the same parent directory as libClearCore and LwIP to properly find include files and libraries.

## Requirements:

### ETHERNET COMMUNICATION:
1. The PC should be running software capable of sending and receiving UDP 
packets. See `udp_client.py` for simple testing.


### MOTOR:
1. A ClearPath motor must be connected to Connector M-0.
2. The connected ClearPath motor must be configured through the MSP software
for Manual Velocity Control mode (In MSP select Mode>>Velocity>>Manual
Velocity Control, then hit the OK button).
3. In the MSP software:
    Define a Max Clockwise and Counter-Clockwise (CW/CCW) Velocity (On the
    main MSP window fill in the textboxes labeled "Max CW Velocity (RPM)"
    and "Max CCW Velocity (RPM)"). Any velocity commanded outside of this
    range will be rejected.
    Set the Velocity Resolution to 2 (On the main MSP window check the
    textbox labeled "Velocity Resolution (RPM per knob count)" 2 is
    default). This means the commanded velocity will always be a multiple
    of 2. For finer resolution, lower this value and change
    velocityResolution in the sketch below to match.
    Set Knob Direction to As-Wired, and check the Has Detents box (On the
    main MSP window check the dropdown labeled "Knob Direction" and the
    checkbox directly below it labeled "Has Detents").
    On the main MSP window set the dropdown labeled "On Enable..." to be
    "Zero Velocity".
    Set the HLFB mode to "ASG-Velocity w/Measured Torque" with a PWM carrier
    frequency of 482 Hz through the MSP software (select Advanced>>High
    Level Feedback \[Mode\]... then choose "ASG-Velocity w/Measured Torque"
    from the dropdown, make sure that 482 Hz is selected in the "PWM Carrier
    Frequency" dropdown, and hit the OK button).

### LIMIT SWITCHES:
1. Limit switches should be connected to the I0 and I1 inputs on the controller.


### EMERGENCY STOP:
1. Emergency stop should be connected to the DI-6 input on the controller.

Links:
ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf


Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT