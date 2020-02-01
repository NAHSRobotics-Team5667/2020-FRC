/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.subsystems.WheelSubsystem;

public class PositionCommand extends CommandBase {
  WheelSubsystem wheelSubsystem;
  /**
   * Creates a new PositionCommand.
   */
  public PositionCommand(WheelSubsystem subsystem, Color targetedColor, Color currentColor) {
    // Use addRequirements() here to declare subsystem dependencies.
    wheelSubsystem = subsystem;
    addRequirements(wheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wheelSubsystem.rotateSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getController().getYButtonPressed()){
      Color m_targetedColor = wheelSubsystem.targetColor();
      Color m_currentColor = wheelSubsystem.getClosestColor();
      
      if (m_targetedColor != m_currentColor) {
        wheelSubsystem.rotateSpeed(0.2);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wheelSubsystem.rotateSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Color targetedColor = wheelSubsystem.targetColor();
    Color currentColor = wheelSubsystem.getClosestColor();
    
    if (targetedColor == currentColor) {
      return true;
    }
    else {
      return false;
    }
  }
}
