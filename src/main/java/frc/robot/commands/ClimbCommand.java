/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends CommandBase {
	ClimbSubsystem m_climbSubsystem;
	// private PIDFController winchController = new PIDFController("Hook height",
	// .01, .01, .01, .01);

	/**
	 * Creates a new ClimbCommand.
	 */
	public ClimbCommand(ClimbSubsystem subsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_climbSubsystem = subsystem;
		addRequirements(m_climbSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_climbSubsystem.lowerHook();
		m_climbSubsystem.stopWinch();

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (RobotContainer.getController().getRightBumperPressed()) {
			m_climbSubsystem.deliverHook();
		} else if (RobotContainer.getController().getLeftBumperPressed()) {
			m_climbSubsystem.lowerHook();
		}

		if (RobotContainer.getController().getDPad() == 0) {
			m_climbSubsystem.driveWinch(.8);
		} else {
			m_climbSubsystem.stopWinch();
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_climbSubsystem.lowerHook();
		m_climbSubsystem.stopWinch();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
