/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autos;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ShootAndAlignCommand;
import frc.robot.commands.shooter.ShootAutonomously;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAndStay extends SequentialCommandGroup {
	/**
	 * Creates a new ShootAndStay.
	 */
	public ShootAndStay(ShooterSubsystem shooter, DriveTrainSubsystem drivetrain, IntakeSubsystem intake,
			Trajectory path, double THRESHOLD, double DEADBAND) {
		super(new ShootAndAlignCommand(drivetrain,
				new ShootAutonomously(shooter, intake, Constants.ShooterConstants.AUTO_LINE_RPM,
						RobotContainer.ballCount, THRESHOLD, DEADBAND)).withTimeout(10),
				RunPath.getCommand(path, drivetrain, false).andThen(new RunCommand(() -> {
					drivetrain.stop();
					shooter.stopFire();
					intake.stopIntakeMotor();
					intake.stopBelt();
				})));
	}
}
