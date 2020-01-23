/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD
import frc.robot.subsystems.SpinnerSubsystem;
=======
import frc.robot.Robot;
import frc.robot.RobotContainer;
>>>>>>> ae244cc2be5a79c2cf5c8aedd530df92a2e7da78

public class PositionControlCommand extends CommandBase {
	SpinnerSubsystem m_SpinnerSubsystem;

	/**
	 * CReates Position Control Command
	 * 
	 * @param spinnerSubsystem - instance of the spinnerSubsystem to be accessed
	 */
	public PositionControlCommand(SpinnerSubsystem spinnerSubsystem) {
		m_SpinnerSubsystem = spinnerSubsystem;
		addRequirements(m_SpinnerSubsystem);

	}

	/**
	 * Starts up the PID controller to find the color
	 */
	@Override
	public void initialize() {
		m_SpinnerSubsystem.position();
		m_SpinnerSubsystem.enable();
	}

	/**
	 * No execute needed for PositionControlCommand
	 */
	@Override
	public void execute() {
	}

	/**
	 * Disables the PID controller when the correct color is reached
	 */
	@Override
	public void end(boolean interrupted) {
		m_SpinnerSubsystem.disable();
	}

	/**
	 * Returns that the PID controller is done
	 */
	@Override
	public boolean isFinished() {
<<<<<<< HEAD
		return m_SpinnerSubsystem.getController().atSetpoint();
=======
		return m_SpinnerSubsystem.isFinished;
>>>>>>> ae244cc2be5a79c2cf5c8aedd530df92a2e7da78
	}
}
