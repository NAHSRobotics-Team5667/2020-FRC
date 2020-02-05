/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

  private IntakeSubsystem m_intake;

  private boolean intakeExtended = false;

  /**
   * Creates a new IntakeCommand.
   */
  public IntakeCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Retract the intake just to be safe
    m_intake.retractIntake();
    intakeExtended = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getController().getBButton() == true) {
      if (intakeExtended == false) {
        m_intake.extendIntake();
      } else if (intakeExtended == true) {
        m_intake.retractIntake();
      }
    }

    if (m_intake.hasSeenBall() == true) {
      m_intake.startBelt();
    } else if (m_intake.hasSeenBall() == false) {
      m_intake.stopBelt();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
