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
import frc.robot.Constants;

public class IntakeCommand extends CommandBase {

  private IntakeSubsystem m_intake;

  private boolean intakeExtended = false;
  private int ballCount = 0;

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
    // Set ballCount to the number of balls that are loaded in to begin
    ballCount = Constants.IntakeConstants.START_BALL_COUNT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Retract/extend intake when B is pressed
    if (RobotContainer.getController().getBButton() == true) {
      if (intakeExtended == false) {
        m_intake.extendIntake();
        intakeExtended = true;
      } else if (intakeExtended == true) {
        m_intake.retractIntake();
        intakeExtended = false;
      }
    }

    // Modify ballCount when balls enter or exit the robot
    if (m_intake.hasSeenBallEnter() == true) {
      ballCount++;
    }
    if (m_intake.hasSeenBallExit() == true) {
      ballCount -= 1;
    }

    // Start/stop the belt based on the ballCount - WIP, make sure to account for
    // every scenario
    if (ballCount < 5 && m_intake.hasSeenBallExit() == true) {
      m_intake.startBelt();
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
