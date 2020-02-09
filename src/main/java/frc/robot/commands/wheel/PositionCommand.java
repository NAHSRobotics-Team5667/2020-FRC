/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.wheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelSubsystem;

/**
 * Creates a new PositionCommand.
 */
public class PositionCommand extends CommandBase {
	WheelSubsystem wheelSubsystem;

	/**
	 * Creates a Position command
	 * 
	 * @param subsystem     - The Wheel Subsystem
	 * @param targetedColor - The target color to stop at
	 * @param currentColor  - The current color seeing
	 */
	public PositionCommand(WheelSubsystem subsystem) {
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
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		wheelSubsystem.rotateSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return wheelSubsystem.targetColor() == wheelSubsystem.getClosestColor();
	}
}
