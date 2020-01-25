/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
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

    public final static class LedConstants {
        public static final int LED_PORT = 4;
        public static final int LED_AMOUNT = 0;
        public static final double FLASH_TIME = 0.5;

        public static enum Colors {
            RED(255, 0, 0), BLUE(0, 0, 255), PURPLE(153, 50, 204), YELLOW(255, 255, 0), WHITE(255, 255, 255),
            GREEN(0, 255, 0);

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

    public final static class AutoConstants {
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

    public final static class DriveConstants {
        public static final double WHEEL_DIAMETER = 0.1524;
        public static final double ENCODER_EDGES_PER_REV = 4096;
        public static final double ENCODER_CONSTANT = (1 / ENCODER_EDGES_PER_REV) * WHEEL_DIAMETER * Math.PI;
        public static final boolean kGyroReversed = false;

        public static final int RIGHT_MASTER = 1;
        public static final int LEFT_MASTER = 0;
        public static final int RIGHT_SLAVE = 3;
        public static final int LEFT_SLAVE = 2;

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

        public static final I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard; // I2C Port value for colorSensor
    }

    public final static class ShooterConstants {
        public static final int RIGHT_SHOOTER_PORT = 4;
        public static final int LEFT_SHOOTER_PORT = 5;
        public static final int ANGLE_SHOOTER_PORT = 6;
        public static final int RIGHT_ENCODER_PORT_A = 8;
        public static final int RIGHT_ENCODER_PORT_B = 9;
        public static final int LEFT_ENCODER_PORT_A = 10;
        public static final int LEFT_ENCODER_PORT_B = 11;
        public static final int ANGLE_ENCODER_PORT_A = 12;
        public static final int ANGLE_ENCODER_PORT_B = 13;
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
