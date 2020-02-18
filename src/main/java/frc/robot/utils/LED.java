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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState.States;

/**
 * Add your docs here.
 */
public class LED {
    private static LED m_led;
    private static AddressableLED m_adressableLed = new AddressableLED(Constants.LedConstants.LED_PORT);
    private static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LedConstants.LED_AMOUNT);

    private int[] colorVals;

    private static Timer m_timer;

    private double cycle = 0;

    private boolean shouldAlternate = false;

    private int[] alternateColor = new int[] { 255, 255, 0 };

    public LED() {
        m_adressableLed.setLength(Constants.LedConstants.LED_AMOUNT);
        m_adressableLed.setData(m_ledBuffer);
        m_adressableLed.start();
        m_timer = new Timer();
        m_timer.reset();
        m_timer.start();
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
        SmartDashboard.putNumber("R", colorVals[0]);
        SmartDashboard.putNumber("G", colorVals[1]);
        SmartDashboard.putNumber("B", colorVals[2]);
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
            breathColor();
        }
        if (Constants.m_RobotState.getCurrentState() == States.DRIVE) {
            oneColor();
        }
        if (Constants.m_RobotState.getCurrentState() == States.SHOOTING) {
            // breathAlternateColor();
            alternateColor();
        }
        if (Constants.m_RobotState.getCurrentState() == States.CLIMBING) {
            getAllianceColor();
            breathColor();
        }
        if (Constants.m_RobotState.getCurrentState() == States.VISION) {
            breathColor();
        }
    }

    /***
     * makes all LEDs the same color
     */
    private void oneColor() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, colorVals[0], colorVals[1], colorVals[2]);
        }

        m_adressableLed.setData(m_ledBuffer);
    }

    private void breathColor() {
        cycle = (cycle + 1) % 360;
        double wave = Math.sin(cycle * 0.0174533);

        for (int i = 0; i < Constants.LedConstants.LED_AMOUNT; i++) {
            int r = Math.abs((int) Math.round(wave * colorVals[0]));
            int g = Math.abs((int) Math.round(wave * colorVals[1]));
            int b = Math.abs((int) Math.round(wave * colorVals[2]));
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_adressableLed.setData(m_ledBuffer);
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

    private void breathAlternateColor() {
        if (cycle % 180 == 0)
            shouldAlternate = !shouldAlternate;
        if (shouldAlternate)
            colorVals = alternateColor;
        breathColor();

    }

    private void alternateColor() {
        cycle = (cycle + 5) % 361;
        double wave = Math.sin(cycle * 0.0174533);

        for (int i = 0; i < Constants.LedConstants.LED_AMOUNT; i++) {
            int r = (int) Math.abs(Math.round((i % 4 != 0 ? colorVals[0] * wave : alternateColor[0] * wave)));
            int g = (int) Math.abs(Math.round((i % 4 != 0 ? colorVals[1] * wave : alternateColor[1] * wave)));
            int b = (int) Math.abs(Math.round((i % 4 != 0 ? colorVals[2] * wave : alternateColor[2] * wave)));
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_adressableLed.setData(m_ledBuffer);
    }

}