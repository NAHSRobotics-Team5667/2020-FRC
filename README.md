# The Digital Eagles Team 5667 2020 Code

The FRC 2020 Infinite Recharge Code

## frc.robot

* [Robot](src/main/java/frc/robot/Robot.java)
* [Constants](src/main/java/frc/robot/Constants.java)
* [RobotContainer](src/main/java/frc/robot/RobotContainer.java)
* [RobotState](src/main/java/frc/robot/RobotState.java)
* [Main](src/main/java/frc/robot/Main.java)

## frc.robot.autos

* [RunPath](src/main/java/frc/robot/autos/RunPath.java) - Command to run a path
* [TrenchPath](src/main/java/frc/robot/autos/TrenchPathAuto.java) - Command that runs path into trench
* [TrenchPathSide](src/main/java/frc/robot/autos/TrenchPathSide.java) - Command that runs path into trench from the side
* [ShootAndStay](src/main/java/frc/robot/autos/ShootAndStay.java) - Command that shoots and crosses line

## frc.robot.subsystems

* [Drivetrain](src/main/java/frc/robot/subsystems/DriveTrainSubsystem.java)
* [Shooter](src/main/java/frc/robot/subsystems/ShooterSubsystem.java)
* [Intake](src/main/java/frc/robot/subsystems/IntakeSubsystem.java)
* [Climb](src/main/java/frc/robot/subsystems/ClimbSubsystem.java)
* [Wheel Manipulator](src/main/java/frc/robot/subsystems/WheelSubsystem.java)

## frc.robot.commands

* actions
  * [Align Command](src/main/java/frc/robot/commands/actions/AlignCommand.java) - Command that aligns to target using computer vision
  * [Hold Position Command](src/main/java/frc/robot/commands/actions/HoldPositionCommand.java) - Command that holds the robot's current position
  * [Turn to Degrees](src/main/java/frc/robot/commands/actions/TurnToDegrees.java) - Command that turns the robot a set amount of degrees
* intake
  * [Intake](src/main/java/frc/robot/commands/intake/IntakeCommand.java)
  * [LoadCommand](src/main/java/frc/robot/commands/intake/LoadCommand.java) - Intake a specific amount of balls
  * [Reset Index Command](src/main/java/frc/robot/commands/intake/ResetIndexCommand.java) - Make sure balls aren't jammed command
* shooter
  * [Shoot and Align](src/main/java/frc/robot/commands/shooter/ShootAndAlignCommand.java) - Shoot and align at the same time
  * [Shoot Automatically](src/main/java/frc/robot/commands/shooter/ShootAutomatically.java) - Controlled shooting based on RPMs
  * [Shoot Autonomously](src/main/java/frc/robot/commands/shooter/ShootAutonomously.java) - Shoot based on amount of balls left in the index
  * [Shoot](src/main/java/frc/robot/commands/shooter/ShooterCommand.java)
* wheel
  * [Position Control](src/main/java/frc/robot/commands/wheel/PositionCommand.java)
  * [Rotation Control](src/main/java/frc/robot/commands/wheel/RotationCommand.java)
* [Drivetrain](src/main/java/frc/robot/commands/DriveTrainCommand.java)
* [Climb](src/main/java/frc/robot/commands/ClimbCommand.java)

## frc.robot.utils

* [PIDFController](src/main/java/frc/robot/utils/PIDFController.java) - A custom PID Controller
* [Controller](src/main/java/frc/robot/utils/Controller.java) - A custom XBox Controller
* [LimeLight](src/main/java/frc/robot/utils/LimeLight.java) - Limelight interface
* [LED](src/main/java/frc/robot/utils/LED.java)
* [Current Spike Counter](src/main/java/frc/robot/utils/CurrentSpikeCounter.java) - Custom counter based on current spikes in a subsystem

## frc.robot.sensors

* [Rev2mTOF](src/main/java/frc/robot/sensors/Rev2mTOF.java)
