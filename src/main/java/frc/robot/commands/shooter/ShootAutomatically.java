/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState.States;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.CurrentSpikeCounter;
import frc.robot.utils.LimeLight;

public class ShootAutomatically extends CommandBase {
	private ShooterSubsystem m_shooter;
	private IntakeSubsystem m_index;
	private CurrentSpikeCounter spike_counter = new CurrentSpikeCounter(Constants.ShooterConstants.AUTO_LINE_THRESHOLD,
			Constants.ShooterConstants.AUTO_LINE_DEADBAND);

	/**
	 * Shoot during teleop and handle index rise
	 */
	public ShootAutomatically(ShooterSubsystem shooter, IntakeSubsystem index) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_shooter = shooter;
		m_index = index;
		addRequirements(m_shooter, m_index);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_shooter.stopFire();
		m_index.reset();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (spike_counter.update(m_shooter.getOutputCurrent(), true)) {
			RobotContainer.ballCount -= 1;
		}

		if (LimeLight.getInstance().getPipeIndex() == 0) {
			// In front of color wheel
			// Shoot using curve if we have a target otherwise assume AUTO Line RPMs
			if (LimeLight.getInstance().hasValidTarget()) {
				m_shooter.curveFire(LimeLight.getInstance().getArea());
			} else {
				m_shooter.fireRPM(Constants.ShooterConstants.AUTO_LINE_RPM);
			}
		} else if (LimeLight.getInstance().getPipeIndex() == 1) {
			// Behind the color wheel
			m_shooter.fireRPM(Constants.ShooterConstants.TRENCH_RPM);
		}

		// Handle index
		if (Constants.m_RobotState.getCurrentState() == States.REVED
				|| RobotContainer.getController().getRightTrigger() >= .1) {
			if (RobotContainer.getController().getLeftTrigger() >= .1) {
				m_index.driveBelt(-.8);
				m_index.setColson(-.3);
			} else {
				m_index.driveBelt(1);
				m_index.setColson(.3);
			}
		} else {
			m_index.stopMotors();
		}

		if (m_shooter.getController().atSetpoint()) {
			Constants.m_RobotState.setState(States.REVED);
		} else {
			if (!(Constants.m_RobotState.getCurrentState() == States.ALIGNING)) {
				Constants.m_RobotState.setState(States.REVING);
			}
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_shooter.stopFire();
		m_index.reset();
		Constants.m_RobotState.setState(States.IDLE);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return RobotContainer.getController().getStickButtonPressed(RobotContainer.getController().getRightHand());
	}
}
