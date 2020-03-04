/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
	private IntakeSubsystem m_intake;

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
		m_intake.retractIntake();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (m_intake.getColsonCurrent() > 20) {
			m_intake.driveBelt(Constants.IntakeConstants.BELT_MOTOR_SPEED);
		} else {
			m_intake.stopBelt();
		}

		if (RobotContainer.getController().getBumper(RobotContainer.getController().getRightHand())) {
			m_intake.driveBelt(.8);
			m_intake.driveIntake(-.3);
			m_intake.setColson(.3);
		} else if (RobotContainer.getController().getBumper(RobotContainer.getController().getLeftHand())) {
			m_intake.driveBelt(-.8);
			m_intake.setColson(-.3);
		} else {
			if (m_intake.isExtended()) {
				m_intake.setColson(.3);
			} else {
				m_intake.stopBelt();
				m_intake.stopColson();
			}
			if (!m_intake.isExtended())
				m_intake.stopIntakeMotor();
		}

		if (RobotContainer.getController().getAButtonPressed()) {
			m_intake.toggle();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_intake.stopBelt();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
