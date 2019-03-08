# Aircon-Controller
Controller for a Mitsubishi Heavy Industries air conditioning unit
Provides mqtt control of the airconditioner using its serial communications link that runs
between MHI's RCD-H-E remote control unit and the CNT port on the air conditioner's control board.
Also provides the ability to control the damper so that airflow can be directed to the correct air conditioing zone.
An inclinometer is provided for the damper in order to detect its position and control the air flow between zones with greater precision.

Requires an mqtt-based controller that measures the temperature readings in each room, compares those temperatures to desired levels and control the airconditioner and damper to provide the optimum comfort.
That controller will be documented with its code in a separate repository.

Photographs and schematics have been included in a specific folder. Note that the damper controller switches high voltages and it should only be deployed by a licensed electrician.
