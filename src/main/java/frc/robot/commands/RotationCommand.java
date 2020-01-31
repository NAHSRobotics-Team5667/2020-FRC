/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SpinnerSubsystem;

public class RotationCommand extends CommandBase {

  private SpinnerSubsystem m_spinner;

  /**
   * Creates a new RotationCommand.
   */
  public RotationCommand(SpinnerSubsystem spinner) {
    m_spinner = spinner;
    addRequirements(m_spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_spinner.control();
    m_spinner.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_spinner.getController().atSetpoint();
  }
}
