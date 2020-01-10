/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
