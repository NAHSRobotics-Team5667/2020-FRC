/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * Add your docs here.
 */
public class CustomRamseteCommand extends RamseteCommand {
    public CustomRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller,
            SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics,
            Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController leftController,
            PIDController rightController, BiConsumer<Double, Double> outputVolts, DriveTrainSubsystem m_drive) {
        super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController,
                outputVolts, m_drive);
        // TODO Auto-generated constructor stub
    }

    public CustomRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController follower,
            DifferentialDriveKinematics kinematics, BiConsumer<Double, Double> outputMetersPerSecond,
            Subsystem[] requirements) {
        super(trajectory, pose, follower, kinematics, outputMetersPerSecond, requirements);
        // TODO Auto-generated constructor stub
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
