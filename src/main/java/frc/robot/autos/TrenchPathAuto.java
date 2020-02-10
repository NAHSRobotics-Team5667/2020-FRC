/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autos;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShootAndHoldCommand;
import frc.robot.commands.ShootAutonomously;
import frc.robot.commands.intake.LoadCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TrenchPathAuto {
	public static SequentialCommandGroup getAuto(Trajectory path, DriveTrainSubsystem drive, IntakeSubsystem intake,
			ShooterSubsystem shooter) {

		RamseteCommand ramseteCommand = RunPath.getCommand(path, drive);

		ShootAutonomously phase1 = new ShootAutonomously(shooter, intake, ShooterConstants.AUTO_LINE_RPM);

		ParallelCommandGroup phase2 = new ParallelCommandGroup(
				new Command[] { ramseteCommand, new InstantCommand(() -> {
					shooter.setSetpoint(ShooterConstants.TRENCH_RPM);
					shooter.enable();
				}), new LoadCommand(intake, 3) });

		ShootAndHoldCommand phase3 = new ShootAndHoldCommand(drive, shooter, intake, ShooterConstants.TRENCH_RPM);

		return new SequentialCommandGroup(new Command[] { phase1, phase2, phase3 });
		// return ramseteCommand.andThen(new RunCommand(() -> m_drive.drive(0, 0,
	}
}
