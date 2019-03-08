# Aircon-Controller
Controller for a Mitsubishi Heavy Industries Model FDC508HES3 air conditioner
Provides mqtt control of the airconditioner using its serial communications link that runs
between MHI's RCD-H-E remote control unit and the CNB port on the air conditioner's control board.
Also provides the ability to control the damper so that airflow can be directed to the correct air conditioning zone.
An inclinometer is provided for the damper in order to detect its position and control the air flow between zones with greater precision.

Requires an mqtt-based controller that measures the temperature readings in each room, compares those temperatures to desired levels and controls the air conditioner and damper to provide the optimum comfort.
That controller will be documented with its code in a separate repository.

Photographs and schematics have been included in a specific folder. Note that the damper controller switches high voltages and it should only be deployed by a licensed electrician.

I've also included a document that shows what I found when reverse engineering the protocol between the RCD-H-E remote control unit and the CNB port on the air conditioner's control board. There are still gaps in some areas of that protocol but all essential functions are covered.

License

This project is licensed under the MIT License - see the LICENSE.md file for details

Acknowledgments

I'd like to acknowledge the work done by Hadley Rich https://github.com/hadleyrich/MQMitsi who did similar work on a Mitsubishi air conditioner that provided me with inspiration to undertake this task.
