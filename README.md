# 2025 REEFSCAPE Robot Code
This code is intended to run on both the 4143 and 4423 robots for the 2025 Season. You can see more about how each robot is configured in the [Robot Configs](#robot-configs) section.

## Controller Bindings
The robots are intended to be driver by two people. One acting as the Driver and the other as Operator. The Driver has control over anything that moves the chassis as well as the entrance/exit of game piece from the robot. The Operator controls the target setpoints for automation as well as elevator/arm movement when in manual control mode. A breakdown for how the controls change across manual/vision and algae/coral mode can be seen in the table below. The overall mapping can be seen in the images below. Anything in Manual/Vision Disable will cause the elevator to move immediately so drivers will need to communicate and be prepared.
| Button | Vision Enable (Coral) | Vision Enable (Algae) | Vision Disable (Coral) | Vision Disable (Algae) |
| ----------- | ---- | ---- | ---- | ---- |
| Driver Left Trigger | Align Scoring Branch | Align Algae (No Game Piece) / Align with Barge orProcessor (Game Piece Present | *Disabled* | *Disabled* |
| Driver Right Trigger | Eject Coral | Eject Algae | <-- <-- | <-- <-- |
| Driver Right Bumper | Coral Station Rotation Align and Pickup | Algae Load | Coral Station Pickup | Algae Load |
| Operator A | Set GSM Target to L1 | <---- | Set Elevator Target to L1 | Set Elevator Target to Barge |
| Operator X | Set GSM Target to L2 | <---- | Set Elevator Target to L2 | Set Elevator Target to Algae High |
| Operator B | Set GSM Target to L3 | <---- | Set Elevator Target to L3 | Set Elevator Target to Algae Low |
| Operator Y | Set GSM Target to L4 | <---- | Set Elevator Target to L4 | Set Elevator Target to Processor |

![Driver Controller](docs/images/driver.jpg)
![Operator Controller](docs/images/operator.jpg)

## Robot Configs
Each Robot has constants that are specific to its design. During construction and upgrades the robot measurements and systems deviated. Some of the major differences are in this table
| Measurement | 4143 | 4423 |
| ----------- | ---- | ---- |
| Left Module Types | MK4N | MK4I |
| Drive Gearing | L3+ | L3 |
| Vision | TBD | Photon |
| Climber | *Present* | *Missing* |
| Coral Pickup | *Present* | *Missing* |


The entire config can be seen in the config files
- [4143 Beta Bot Config](/src/main/deploy/robots/BetaBot.json)
- [4423 Alpha Bot Config](/src/main/deploy/robots/AlphaBot.json)
