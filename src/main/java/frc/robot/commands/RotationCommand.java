/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SpinnerSubsystem;

public class RotationCommand extends CommandBase {
	PIDController vController = new PIDController(0, 0, 0);
	ShuffleboardTab tab = Shuffleboard.getTab("Wheel");

	private SpinnerSubsystem m_spinner;

	/**
	 * Creates a new RotationCommand.
	 */
	public RotationCommand(SpinnerSubsystem spinner) {
		m_spinner = spinner;
		addRequirements(m_spinner);
		tab.add(vController);
		tab.addNumber("v_error", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return vController.getVelocityError();
			}
		}).withWidget("Graph");
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// m_spinner.control();
		// m_spinner.reset();
		vController.setSetpoint(40);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// m_spinner.drive(RobotContainer.getController().getLeftY());
		// if (RobotContainer.getController().getBButton()) {
		// m_spinner.drive(vController.calculate(m_spinner.getCurrentRPM()));
		// }
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// m_spinner.disable();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return m_spinner.getController().atSetpoint();
		// return m_spinner.hasRotated();
		return false;
	}
}
