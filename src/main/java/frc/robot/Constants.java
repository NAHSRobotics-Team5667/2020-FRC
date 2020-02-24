/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static RobotState m_RobotState = new RobotState(null);

    public final static class LedConstants {
        public static final int LED_PORT = 0;
        public static final int LED_AMOUNT = 178;
        public static final double FLASH_TIME = 2;

        public static enum Colors {
            RED(255, 0, 0), BLUE(0, 0, 255), PURPLE(25, 0, 51), YELLOW(255, 255, 0), WHITE(255, 255, 255),
            GREEN(0, 255, 0), MAROON(69, 0, 0), GOLD(128, 128, 0), PINK(255, 0, 178);

            private int r, g, b;

            private Colors(int r, int g, int b) {
                this.r = r;
                this.g = g;
                this.b = b;
            }

            public int[] getColor() {
                return new int[] { r, g, b };
            }
        }
    }

    public final static class IntakeConstants {
        public static final double INTAKE_MOTOR_SPEED = .9;
        public static final double BELT_MOTOR_SPEED = .25;

        public static final boolean SOLENOID_FIRED = true;
        public static final double SENSOR_RANGE_INCHES = 5;
        public static final int START_BALL_COUNT = 3;

        public static final int MOTOR_PORT = 1;
        public static final int BELT_PORT = 2;

        public static final int SOLENOID_PORT = 0;
        public static final int SOLENOID_2_PORT = 7;
    }

    public final static class ShooterConstants {
        public static final int PORT = 4;

        public static final double AUTO_LINE_RPM = 3750;
        public static final double TRENCH_RPM = 4250;

        public static final int IDLE_VOLTAGE = 3;

        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.0613 / 70;
        public static final double kaVoltSecondsSquaredPerMeter = 0.00831 / 100;
        public static final double kP = 0.9 / 120;
        public static final double kI = kP / 4;
        public static final double kD = 0;
        public static final double ENCODER_CONSTANT = (2.0 / 2048.0);
        public static final double SPIKE = 40;
        public static final double SPIKE_TIME = 2;
    }

    public final static class WheelConstants {
        public static final double ROTATIONS = 2.7 * 3;
        public static final int MOTOR = 5;
        public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard; // I2C Port value for colorSensor
    }

    public final static class ClimbConstants {
        public static final int ENCODER_CONSTANT = 1;
    }

    public final static class VisionConstants {
        public static final double H1 = Units.inchesToMeters(36); // Height of limelight from the ground
        public static final double H2 = Units.inchesToMeters(98.25); // Height of target
        public static final double A1 = 10; // Limelight mounting angle
        public static final double kP = 0.015;
        public static final double kI = 0.02;
        public static final double kD = 0;
    }

    public final static class DriveConstants {
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(6);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER * Math.PI;
        public static final double ENCODER_EDGES_PER_REV = 21934;
        public static final double GEAR_RATIO = 10.71;
        public static double MAG = -1;
        public static final double ENCODER_CONSTANT = MAG * (1 / ENCODER_EDGES_PER_REV) * WHEEL_DIAMETER * Math.PI;
        public static final boolean kGyroReversed = true;

        public static final double MAX_SPEED_TELE = 3.25;
        public static final double MAX_ANGULAR_VEL = 320;

        public static final int RIGHT_MASTER = 1;
        public static final int LEFT_MASTER = 3;
        public static final int RIGHT_SLAVE = 0;
        public static final int LEFT_SLAVE = 2;

        public static final double ksVolts = 0.0869 / 10;
        public static final double kvVoltSecondsPerMeter = 2.46 / 10;
        public static final double kaVoltSecondsSquaredPerMeter = 0.185 / 10; // 1.9356652467050692

        public static final double kTrackwidthMeters = Units.inchesToMeters(22);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static double kP = 0.01; // 7.43;
        public static double kI = 0; // 0.01; // 7.43;
        public static double kD = 0; // 0.01; // 7.43;

    }

    public final static class AutoConstants {
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final PIDController L_CONTROLLER = new PIDController(DriveConstants.kP, DriveConstants.kI,
                DriveConstants.kD);
        public static final PIDController R_CONTROLLER = new PIDController(DriveConstants.kP, DriveConstants.kD,
                DriveConstants.kD);

    }

    public final static class PATHS {
        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 6);

        // Create config for trajectory
        public static final TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.
        public static final Trajectory S_TRAJECTORY = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        public static final Trajectory STRAIGHT_TRAJECTORY_2M = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints
                List.of(new Translation2d(1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                // Pass config
                config);

        public static final class PathWeaver {
            public static Trajectory getTrajectory(String path) {
                try {
                    return TrajectoryUtil
                            .fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/" + path + ".wpilib.json"));
                } catch (Exception e) {
                    System.out.println("CANNOT READ Trajectory - " + path);
                    System.out.println("WITH ERROR: " + e.toString());
                    return null;
                }
            }
        }
    }

    public final static class ControllerConstants {
        public static final int CONTROLLER_PORT = 0; // Controller port

        // Sticks
        public static final int S_RIGHT_X_PORT = 4; // Right stick x
        public static final int S_RIGHT_Y_PORT = 5; // Right stick y
        public static final int S_LEFT_X_PORT = 0; // Left stick x
        public static final int S_LEFT_Y_PORT = 1; // Left stick y

        // Triggers
        public static final int TRIGGER_RIGHT_PORT = 3; // Right trigger
        public static final int TRIGGER_LEFT_PORT = 2; // Left trigger

        // Bumpers
        public static final int BUMPER_RIGHT_PORT = 6; // Right bumper
        public static final int BUMPER_LEFT_PORT = 5; // Left bumper

        // Buttons
        public static final int BUTTON_A_PORT = 1; // A Button
        public static final int BUTTON_B_PORT = 2; // B Button
        public static final int BUTTON_X_PORT = 3; // X Button
        public static final int BUTTON_Y_PORT = 4; // Y Button

        // Special buttons
        public static final int BUTTON_MENU_PORT = 8; // Menu Button
        public static final int BUTTON_START_PORT = 7; // Start button

    }
}
