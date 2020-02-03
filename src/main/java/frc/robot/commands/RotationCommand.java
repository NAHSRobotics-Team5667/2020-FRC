/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//Countdown timer?
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelSubsystem;

/**
 * Creates a new RotationCommand.
 */
public class RotationCommand extends CommandBase {

	WheelSubsystem wheelSubsystem;

	/**
	 * Create a rotation command
	 * 
	 * @param subsystem The Wheel Subsystem
	 */
	public RotationCommand(WheelSubsystem subsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		wheelSubsystem = subsystem;
		addRequirements(wheelSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		wheelSubsystem.rotateSpeed(0.2);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		/*
		 * Haikus with Olu:
		 *
		 * While "X" button's held... then spin the motor so long... the "X" button's
		 * pressed.
		 * 
		 * This has been Haikus with Olu.
		 */
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		wheelSubsystem.rotateSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return wheelSubsystem.hasRotated();
	}
}
