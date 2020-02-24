/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.actions;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.DriveModes;
import frc.robot.utils.LimeLight;
import frc.robot.utils.PIDFController;

public class AlignCommand extends CommandBase {
	private DriveTrainSubsystem m_drive;
	private boolean isAutoAligning = true;
	private PIDFController angleController = new PIDFController("Angle", Constants.VisionConstants.kP,
			Constants.VisionConstants.kI, Constants.VisionConstants.kD, 0);
	private ShuffleboardTab alignmentTab = Shuffleboard.getTab("Auto Alignment");

	/**
	 * Creates a new AlignCommand.
	 */
	public AlignCommand(DriveTrainSubsystem drive) {
		System.out.println("STARTING ALIGN COMMAND");
		// Use addRequirements() here to declare subsystem dependencies.
		m_drive = drive;
		addRequirements(m_drive);
		// alignmentTab.add("Angle Controller", angleController);

		// alignmentTab.addNumber("setpoint", new DoubleSupplier() {
		// @Override
		// public double getAsDouble() {
		// return LimeLight.getInstance().getXAngle();
		// }
		// });

		// alignmentTab.addNumber("output", new DoubleSupplier() {
		// @Override
		// public double getAsDouble() {
		// return angleController.getPositionError();
		// }
		// });

		// alignmentTab.addBoolean("isAligning", new BooleanSupplier() {

		// @Override
		// public boolean getAsBoolean() {
		// return isAutoAligning;
		// }
		// });

		angleController.setTolerance(1, 3);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		LimeLight.getInstance().turnLightOn();
		m_drive.setDriveMode(DriveTrainSubsystem.DriveModes.AUTO);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_drive.getDriveMode() == DriveTrainSubsystem.DriveModes.AUTO && LimeLight.getInstance().hasValidTarget()) {
			double angle = -angleController.calculate(LimeLight.getInstance().getXAngle());
			double output = Constants.DriveConstants.ksVolts + angle;
			m_drive.tankDriveVolts(output, -output);
			m_drive.feedMotorSafety();

		} else {
			m_drive.feedMotorSafety();
			m_drive.stop();
		}

		Map<String, Double> sticks = RobotContainer.getController().getSticks();
		if (Math.abs(sticks.get("LSX")) > .2 || Math.abs(sticks.get("LSY")) > .2 || Math.abs(sticks.get("RSX")) > .2
				|| Math.abs(sticks.get("RSY")) > .2) {
			m_drive.setDriveMode(DriveModes.MANUAL);
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_drive.stop();
		m_drive.setDriveMode(DriveModes.MANUAL);
		isAutoAligning = false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (m_drive.getDriveMode() == DriveModes.MANUAL) {
			System.out.println("ENDED BC MANUAL");
			return true;
		} else if (!LimeLight.getInstance().hasValidTarget()) {
			System.out.println("ENDED BC NO TARGET");
			return true;
		} else if (angleController.atSetpoint()) {
			System.out.println("ENDED BC AT ANGLE");
			return true;
		} else {
			return false;
		}
	}
}
