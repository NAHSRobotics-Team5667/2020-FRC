/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.AlignCommand;
import frc.robot.commands.actions.HoldPositionCommand;
import frc.robot.commands.shooter.ShootAutonomously;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAndHoldCommand extends SequentialCommandGroup {

  /**
   * Creates a new Shoot Command that holds & aligns the position using the
   * limelight
   * 
   * @param drive     - The DriveTrain Subsystem
   * @param shooter   - The Shooter Subsystem
   * @param intake    - The Intake Subsystem
   * @param targetRPM - The target RPM when shooting
   */
  public ShootAndHoldCommand(DriveTrainSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake,
      double targetRPM) {
    // Align -> Hold -> Shoot
    super(new AlignCommand(drive),
        new HoldPositionCommand(drive).alongWith(new ShootAutonomously(shooter, intake, targetRPM)));
  }
}
