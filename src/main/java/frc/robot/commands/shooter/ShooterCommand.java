/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.LimeLight;

public class ShooterCommand extends CommandBase {
	private ShooterSubsystem m_shooter;
	private PIDController pidController = new PIDController(Constants.ShooterConstants.kP, 0,
			Constants.ShooterConstants.kD);

	/**
	 * Creates a new ShooterCommand.
	 */
	public ShooterCommand(ShooterSubsystem shooter) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_shooter = shooter;
		addRequirements(m_shooter);
		Shuffleboard.getTab("Teleop").add("Shooter PID", pidController);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_shooter.stopFire();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (RobotContainer.getController().getXButton()) {
			if (LimeLight.getInstance().getPipeIndex() == 0) {
				// pidController.setSetpoint(Constants.ShooterConstants.AUTO_LINE_RPM);
				m_shooter.fireRPM(6000);
			} else {
				// pidController.setSetpoint(Constants.ShooterConstants.TRENCH_RPM);
				m_shooter.fireRPM(Constants.ShooterConstants.TRENCH_RPM);
			}
		} else {
			m_shooter.resetIError();
			m_shooter.fire(RobotContainer.getController().getRightTrigger());
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_shooter.stopFire();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
