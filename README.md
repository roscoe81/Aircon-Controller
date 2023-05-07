# Aircon-Controller
Controller for a Mitsubishi air conditioner (Model FDC508HES3)
Provides mqtt control of the airconditioner using the serial communications link that runs
between Mitsubishi's RCD-H-E remote control unit and the CNB port on the air conditioner's control board.
Also provides the ability to control a damper so that airflow can be directed to the correct air conditioning zone.
An inclinometer is provided for the damper in order to detect its position and control the air flow between zones with greater precision.

This project requires a separate mqtt-based controller that captures the actual temperatures of each room, compares those temperatures to desired temperature levels and controls the air conditioner and damper to align the desired and actual temperatures. I've implemented such a [controller](https://github.com/roscoe81/Home-Manager) as part of a more comprehensive Home Automation System.

Version 5.1 Gen adds the ability to control dampers for each room and to control fan speed. Hardware changes were necessary to control each room damper.

## Hardware Schematics
![Aircon Controller Schematic](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/Aircon%20Controller_schem.png)

![Damper Position Sensor Schematic](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/Aircon%20Damper%20Position%20Sensor_schem.png)

![Damper Control Board Schematic](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/Aircon%20Damper%20Control%20Board_schem.png)

## Hardware Photographs
![Aircon Controller](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/IMG_1565.png)

![Damper Position Sensor](https://github.com/roscoe81/Aircon-Controller/blob/master/Schematics%20and%20Photos/IMG_1432.png)

**Note that the damper controller switches high voltages and it should only be constructed and deployed by a licensed electrician.**

## Air Conditioner Communications Protocol
I've included a [document](https://github.com/roscoe81/Aircon-Controller/blob/master/Serial%20Comms/Aircon%20Controller%20Comms.pdf) that shows what I found when reverse engineering the protocol between the Mitsubishi RCD-H-E remote control unit and the CNB port on the air conditioner's main unit. There are still gaps in some areas of that protocol but all essential functions are covered.

![Communications Protocol](https://github.com/roscoe81/Aircon-Controller/blob/master/Serial%20Comms/F43B66E8-F1EA-4F13-B185-9C76222CD1DF.jpeg)

## License

This project is licensed under the MIT License - see the LICENSE.md file for details

## Acknowledgements

I'd like to acknowledge the work done by [Hadley Rich](https://github.com/hadleyrich/MQMitsi) who did similar work on a Mitsubishi air conditioner that provided me with the inspiration to undertake this task.
