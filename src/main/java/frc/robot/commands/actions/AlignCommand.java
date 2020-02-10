/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.actions;

import java.util.Map;
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

	private PIDFController angleController = new PIDFController("Angle", 0.02, 0, 0.001, 0);
	private ShuffleboardTab alignmentTab = Shuffleboard.getTab("Auto Alignment");

	/**
	 * Creates a new AlignCommand.
	 */
	public AlignCommand(DriveTrainSubsystem drive) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_drive = drive;
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
		LimeLight.getInstance().turnLightOn();
		m_drive.setDriveMode(DriveTrainSubsystem.DriveModes.AUTO);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_drive.getDriveMode() == DriveTrainSubsystem.DriveModes.AUTO && LimeLight.getInstance().hasValidTarget()) {

			double angle = -angleController.calculate(LimeLight.getInstance().getXAngle());
			double output = (Constants.DriveConstants.ksVolts * Math.signum(angle)) + angle;
			m_drive.tankDriveVolts(output, -output);
			m_drive.feedMotorSafety();

		}

		if (RobotContainer.getController().getSticks().equals(Map.of("LSX", 0, "LSY", 0, "RSX", 0, "RSY", 0))) {
			m_drive.setDriveMode(DriveModes.MANUAL);
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		LimeLight.getInstance().turnLightOff();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_drive.getDriveMode() == DriveModes.MANUAL || !LimeLight.getInstance().hasValidTarget()
				|| angleController.atSetpoint();
	}
}
