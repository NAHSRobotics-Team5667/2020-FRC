/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState.States;

/**
 * Add your docs here.
 */
public class LED {
    private static LED m_led;
    private static AddressableLED m_adressableLed = new AddressableLED(Constants.LedConstants.ledPort);
    private static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.ledAmount);

    private int[] colorVals;

    private static Timer m_timer;

    public LED() {
        m_adressableLed.setLength(m_ledBuffer.getLength());
        m_adressableLed.setData(m_ledBuffer);
        m_adressableLed.start();
    }

    /***
     * 
     * @return the LED Instance
     */
    public static LED getLEDInstance() {
        if (m_led == null) {
            m_led = new LED();
        }
        return m_led;

    }

    /***
     * Gets the color based on the current state
     */
    private void getColor() {
        colorVals = Constants.m_RobotState.getCurrentState().getColor();
    }

    /***
     * determines what to set the LED to based off of current state
     */
    public void setColor() {
        getColor();
        if (Constants.m_RobotState.getCurrentState() == States.IDLE) {
            getAllianceColor();
            oneColor();
        }
        if (Constants.m_RobotState.getCurrentState() == States.AUTO) {
            flashColor();
        }
        if (Constants.m_RobotState.getCurrentState() == States.DRIVE) {
            oneColor();
        }
        if (Constants.m_RobotState.getCurrentState() == States.SHOOTING) {
            flashAndAlternate();
        }
        if (Constants.m_RobotState.getCurrentState() == States.CLIMBING) {
            getAllianceColor();
            flashColor();
        }

    }

    /***
     * makes all LEDs the same color
     */
    private void oneColor() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, colorVals[0], colorVals[1], colorVals[0]);
        }

        m_adressableLed.setData(m_ledBuffer);
    }

    /***
     * flashes a color on and off
     */
    private void flashColor() {
        m_timer.reset();
        m_timer.start();
        if (m_timer.hasPeriodPassed(Constants.LedConstants.flashTime)) {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, colorVals[0], colorVals[1], colorVals[0]);
            }
            m_adressableLed.setData(m_ledBuffer);
        }
    }

    /***
     * set colorVals to alliance color
     */
    private void getAllianceColor() {
        if (DriverStation.getInstance().getAlliance() == Alliance.Blue) {
            colorVals = new int[] { 0, 0, 255 };
        } else if (DriverStation.getInstance().getAlliance() == Alliance.Red) {
            colorVals = new int[] { 255, 0, 0 };
        }
    }

    /***
     * alternates color of LEDs on strip and flashes all LEDs
     */
    private void flashAndAlternate() {
        m_timer.reset();
        m_timer.start();
        if (m_timer.hasPeriodPassed(Constants.LedConstants.flashTime)) {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                if (i % 2 == 0) {
                    m_ledBuffer.setRGB(i, colorVals[0], colorVals[1], colorVals[0]);
                } else {
                    m_ledBuffer.setRGB(i, 255, 255, 255);
                }

            }
            m_adressableLed.setData(m_ledBuffer);
        }
    }

}
