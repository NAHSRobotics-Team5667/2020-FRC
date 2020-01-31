/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

/**
 * LED Singleton class
 */
public class LED {
    private static LED m_led;
    private SerialPort serialPort;
    private int colorState;

    public LED() {
        serialPort = new SerialPort(9600, SerialPort.Port.kUSB);
    }

    /**
     * Get the LED instance
     * 
     * @return the LED Instance
     */
    public static LED getLEDInstance() {
        if (m_led == null) {
            m_led = new LED();
        }
        return m_led;

    }

    /**
     * Sets the current color by communicating with the arduino
     */
    public void setColor() {
        colorState = Constants.m_RobotState.getCurrentState().getState();
        serialPort.writeString(Integer.toString(colorState));
    }

}
