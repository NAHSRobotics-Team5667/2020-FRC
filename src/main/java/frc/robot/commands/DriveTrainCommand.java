/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.States;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utils.LimeLight;
import frc.robot.utils.PIDFController;

public class DriveTrainCommand extends CommandBase {

	private PIDFController angleController = new PIDFController("Angle", 0.02, 0, 0.001, 0);
	private ShuffleboardTab alignmentTab = Shuffleboard.getTab("Auto Alignment");
	private DriveTrainSubsystem m_drive;

	/**
	 * Create a Drive Train Subsystem
	 * 
	 * @param DriveTrain - The Drive Train Subsystem
	 */
	public DriveTrainCommand(DriveTrainSubsystem DriveTrain) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_drive = DriveTrain;
		addRequirements(m_drive);
		alignmentTab.add(angleController);
		alignmentTab.addNumber("setpoint", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return LimeLight.getInstance().getXAngle();
			}
		});

		alignmentTab.addNumber("output", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return angleController.getPositionError();
			}
		});

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_drive.stop();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Map<String, Double> sticks = RobotContainer.getController().getSticks();

		if (RobotContainer.getController().getYButton()) {
			LimeLight.getInstance().turnLightOn();
			m_drive.setDriveMode(DriveTrainSubsystem.DriveModes.AUTO);
		} else {
			// LimeLight.getInstance().turnLightOff();
			m_drive.setDriveMode(DriveTrainSubsystem.DriveModes.MANUAL);
		}

		if (m_drive.getDriveMode() == DriveTrainSubsystem.DriveModes.AUTO && LimeLight.getInstance().hasValidTarget()) {

			double angle = -angleController.calculate(LimeLight.getInstance().getXAngle());
			double output = (Constants.DriveConstants.ksVolts * Math.signum(angle)) + angle;
			m_drive.tankDriveVolts(output, -output);
			m_drive.feedMotorSafety();

		} else {
			// Drive using joysticks
			m_drive.drive(sticks.get("LSY"), sticks.get("RSX"),
					RobotContainer.getController().getStickButtonPressed(RobotContainer.getController().getLeftHand()));
		}
		if (Constants.m_RobotState.getCurrentState() != States.ROTATION
				&& Constants.m_RobotState.getCurrentState() != States.SHOOTING
				&& Constants.m_RobotState.getCurrentState() != States.POSITION) {
			Constants.m_RobotState.setState(States.DRIVE);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_drive.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

}
