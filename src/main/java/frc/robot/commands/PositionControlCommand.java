/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class PositionControlCommand extends CommandBase {
  SpinnerSubsystem m_SpinnerSubsystem;

  /**
   * Creates a new Posit ionControlCommand.
   */

  public PositionControlCommand(SpinnerSubsystem spinnerSubsystem) {
    m_SpinnerSubsystem = spinnerSubsystem;
    addRequirements(m_SpinnerSubsystem);

  }

  @Override
  public void initialize() {
    m_SpinnerSubsystem.position();
    m_SpinnerSubsystem.enable();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_SpinnerSubsystem.disable();
  }

  @Override
  public boolean isFinished() {
    return m_SpinnerSubsystem.isFinished;
  }
}
