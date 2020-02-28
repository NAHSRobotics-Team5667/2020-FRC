/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autos;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.PATHS.PathWeaver;
import frc.robot.commands.shooter.ShootAndAlignCommand;
import frc.robot.commands.shooter.ShootAutonomously;
import frc.robot.commands.actions.AlignCommand;
import frc.robot.commands.actions.HoldPositionCommand;
import frc.robot.commands.intake.LoadCommand;
import frc.robot.commands.intake.ResetIndexCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.LimeLight;

public class TrenchPathAuto {
	public static SequentialCommandGroup getAuto(Trajectory path, Trajectory trenchPath, DriveTrainSubsystem drive,
			IntakeSubsystem intake, ShooterSubsystem shooter) {

		RobotContainer.ballCount = 3;

		drive.stop();
		drive.resetOdometry(path.getInitialPose());
		shooter.stopFire();
		LimeLight.getInstance().setPipeline(0);

		RamseteCommand toTrench = RunPath.getCommand(path, drive, false);
		RamseteCommand trenchToWheel = RunPath.getCommand(trenchPath, drive, false);

		SequentialCommandGroup phase1 = new ShootAndAlignCommand(drive,
				new ShootAutonomously(shooter, intake, ShooterConstants.AUTO_LINE_RPM, RobotContainer.ballCount))
						.andThen(new InstantCommand(() -> {
							drive.stop();
							shooter.stopFire();
						}));

		SequentialCommandGroup phase2 = new SequentialCommandGroup(new Command[] { toTrench.andThen(() -> {
			LimeLight.getInstance().setPipeline(1);
		}), new ParallelCommandGroup(new LoadCommand(intake, 1),
				trenchToWheel.andThen(new InstantCommand(drive::stop))) }).andThen(new InstantCommand(() -> {
					drive.setNeutralMode(NeutralMode.Brake);
					drive.stop();
					shooter.stopFire();
				}));

		// ResetIndexCommand resetIndex = new ResetIndexCommand(intake, shooter);

		return new SequentialCommandGroup(new Command[] {
				// Phase 1
				phase1,
				// Phase 2
				phase2,
				// Phase 3
				new ShootAndAlignCommand(drive, new ShootAutonomously(shooter, intake, ShooterConstants.TRENCH_END_RPM,
						RobotContainer.ballCount)) }).andThen(new RunCommand(() -> {
							drive.setNeutralMode(NeutralMode.Brake);
							drive.stop();
							shooter.stopFire();
							intake.stopBelt();
							intake.stopIntakeMotor();
						}));
	}
}
