/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//Countdown timer?
package frc.robot.commands.wheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState.States;
import frc.robot.subsystems.WheelSubsystem;

/**
 * Creates a new RotationCommand.
 */
public class RotationCommand extends CommandBase {

  WheelSubsystem wheelSubsystem;

  /**
   * Create a rotation command
   * 
   * @param subsystem The Wheel Subsystem
   */
  public RotationCommand(WheelSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    wheelSubsystem = subsystem;
    addRequirements(wheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.m_RobotState.setState(States.ROTATION);
    wheelSubsystem.rotateSpeed(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.m_RobotState.setState(States.IDLE);
    wheelSubsystem.turnOffMotor();
    wheelSubsystem.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wheelSubsystem.hasRotated();
  }
}
