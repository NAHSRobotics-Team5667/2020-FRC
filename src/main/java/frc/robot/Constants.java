/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

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

    public final static class AutoConstants {
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

    public final static class DriveConstants {
        public static final double WHEEL_DIAMETER = 0.1524;
        public static final double ENCODER_EDGES_PER_REV = 4096;
        public static final double encoderConstant = (1 / ENCODER_EDGES_PER_REV) * WHEEL_DIAMETER * Math.PI;
        public static final boolean kGyroReversed = false;

        public static final int rightMaster = 1;
        public static final int leftMaster = 0;
        public static final int rightSlave = 3;
        public static final int leftSlave = 2;

        public static final double ksVolts = .519;
        public static final double kvVoltSecondsPerMeter = 3.15;
        public static final double kaVoltSecondsSquaredPerMeter = .491;

        public static final double kTrackwidthMeters = Units.inchesToMeters(15);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final double kPDrive = 37.6;
        public static final double kDDrive = 16.7;

    }

    public final static class VisionConstants {
        public static final double H1 = 0; // Height of limelight from the ground
        public static final double H2 = 0; // Height of target
        public static final double A1 = 0; // Limelight mounting angle
    }

    public final static class ShooterConstants {
        public static final int RightShooter_Port = 4;
        public static final int LeftShooter_Port = 5;
        public static final int AngleShooter_Port = 6;
        public static final int RightEncoder_Port_A = 8;
        public static final int RightEncoder_Port_B = 9;
        public static final int LeftEncoder_Port_A = 10;
        public static final int LeftEncoder_Port_B = 11;
        public static final int AngleEncoder_Port_A = 12;
        public static final int AngleEncoder_Port_B = 13;
    }

    public final static class ControllerConstants {
        public static final int controllerPort = 0; // Controller port

        // Sticks
        public static final int sRightX_Port = 4; // Right stick x
        public static final int sRightY_Port = 5; // Right stick y
        public static final int sLeftX_Port = 0; // Left stick x
        public static final int sLeftY_Port = 1; // Left stick y

        // Triggers
        public static final int TriggerRight_Port = 3; // Right trigger
        public static final int TriggerLeft_Port = 2; // Left trigger

        // Bumpers
        public static final int BumperRight_Port = 6; // Right bumper
        public static final int BumperLeft_Port = 5; // Left bumper

        // Buttons
        public static final int button_A_Port = 1; // A Button
        public static final int button_B_Port = 2; // B Button
        public static final int button_X_Port = 3; // X Button
        public static final int button_Y_Port = 4; // Y Button

        // Special buttons
        public static final int button_Menu_Port = 8; // Menu Button
        public static final int button_Start_Port = 7; // Start button

    }
}
