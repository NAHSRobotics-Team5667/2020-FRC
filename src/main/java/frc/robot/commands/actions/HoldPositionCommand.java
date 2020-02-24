/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class HoldPositionCommand extends CommandBase {
	private DriveTrainSubsystem m_drive;

	/**
	 * Creates a new HoldPositionCommand.
	 */
	public HoldPositionCommand(DriveTrainSubsystem drive) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_drive = drive;
		addRequirements(m_drive);
		System.out.println("START HOLD");
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		Constants.AutoConstants.L_CONTROLLER.setSetpoint(m_drive.getLeftEncoderPosition());
		Constants.AutoConstants.R_CONTROLLER.setSetpoint(m_drive.getRightEncoderPosition());

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_drive.tankDriveVolts(Constants.AutoConstants.L_CONTROLLER.calculate(m_drive.getLeftEncoderPosition()),
				Constants.AutoConstants.R_CONTROLLER.calculate(m_drive.getRightEncoderPosition()));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_drive.stop();
		System.out.println("END HOLD");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
