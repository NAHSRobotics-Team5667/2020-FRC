/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState.States;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.CurrentSpikeCounter;
import frc.robot.utils.LimeLight;

public class ShooterCommand extends CommandBase {
	private ShooterSubsystem m_shooter;
	private CurrentSpikeCounter spike_counter = new CurrentSpikeCounter(Constants.ShooterConstants.AUTO_LINE_THRESHOLD,
			Constants.ShooterConstants.AUTO_LINE_DEADBAND);

	/**
	 * Creates a new ShooterCommand.
	 */
	public ShooterCommand(ShooterSubsystem shooter) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_shooter = shooter;
		addRequirements(m_shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_shooter.stopFire();
		m_shooter.resetIError();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (spike_counter.update(m_shooter.getOutputCurrent(), true)) {
			RobotContainer.ballCount -= 1;
		}
		if (RobotContainer.getController().getXButton()) {
			if (!LimeLight.getInstance().hasValidTarget()) {
				if (m_shooter.getController().atSetpoint()
						&& Constants.m_RobotState.getCurrentState() == States.ALIGNED) {
					Constants.m_RobotState.setState(States.REVED);
				} else {
					Constants.m_RobotState.setState(States.REVING);
				}

				if (LimeLight.getInstance().getPipeIndex() == 0) {
					m_shooter.fireRPM(Constants.ShooterConstants.AUTO_LINE_RPM);
				} else {
					m_shooter.fireRPM(Constants.ShooterConstants.TRENCH_FAR_RPM);
				}

			} else {
				if (LimeLight.getInstance().getPipeIndex() == 0) {
					double x = LimeLight.getInstance().getArea();
					m_shooter.curveFire(x);
				} else {
					m_shooter.fireRPM(Constants.ShooterConstants.TRENCH_FAR_RPM);
				}
				if (!m_shooter.getController().atSetpoint()
						&& (Constants.m_RobotState.getCurrentState() == States.ALIGNED
								|| Constants.m_RobotState.getCurrentState() == States.REVING)) {
					Constants.m_RobotState.setState(States.REVING);
				} else if (m_shooter.getController().atSetpoint()) {
					Constants.m_RobotState.setState(States.REVED);
				}
			}
		} else {
			if (RobotContainer.getController().getLeftTrigger() > .1) {
				m_shooter.fire(-1);
			} else {
				m_shooter.stopFire();
				if ((Constants.m_RobotState.getCurrentState() == States.REVING
						|| Constants.m_RobotState.getCurrentState() == States.REVED)
						&& Constants.m_RobotState.getCurrentState() != States.CLIMBING) {
					Constants.m_RobotState.setState(States.IDLE);
				}
			}
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_shooter.stopFire();
		Constants.m_RobotState.setState(States.IDLE);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
