/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.CurrentSpikeCounter;
import frc.robot.utils.LimeLight;

public class ShootAutonomously extends CommandBase {
	private ShooterSubsystem m_shooter;
	private IntakeSubsystem m_intake;
	private double targetRPM;
	private int target_shots;

	private boolean location = LimeLight.getInstance().getPipeIndex() == 0;
	private boolean hasRamped = false;

	private CurrentSpikeCounter spikeCounter = new CurrentSpikeCounter(
			location ? Constants.ShooterConstants.AUTO_LINE_THRESHOLD : Constants.ShooterConstants.TRENCH_THRESHOLD,
			location ? Constants.ShooterConstants.AUTO_LINE_DEADBAND : Constants.ShooterConstants.TRENCH_DEADBAND);

	/**
	 * Creates a new ShootAuto.
	 */
	public ShootAutonomously(ShooterSubsystem shooter, IntakeSubsystem intake, double targetRPM, int amt) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_shooter = shooter;
		m_intake = intake;

		addRequirements(m_shooter, m_intake);

		this.targetRPM = targetRPM;
		target_shots = RobotContainer.ballCount - amt;

		System.out.println("STARTING SHOOT");

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_intake.retractIntake();
		m_intake.stopBelt();
		m_shooter.resetIError();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (!hasRamped && m_shooter.getController().atSetpoint()) {
			hasRamped = true;
		}

		if (spikeCounter.update(m_shooter.getOutputCurrent())) {
			RobotContainer.ballCount -= 1;
		}

		m_shooter.fireRPM(targetRPM);
		if (m_shooter.getController().atSetpoint()) {
			m_intake.driveBelt(.6);
			m_intake.driveIntake(-0.4);
		} else {
			m_intake.stopBelt();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_intake.stopBelt();
		m_intake.stopIntakeMotor();
		m_shooter.stopFire();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return RobotContainer.ballCount <= 0 || RobotContainer.ballCount == target_shots;
	}
}
