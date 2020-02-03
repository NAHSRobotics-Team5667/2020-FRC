/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WheelSubsystem extends SubsystemBase {
	private WPI_TalonFX m_motor;
	private ColorSensorV3 m_colorV3;
	private ColorMatch m_colorMatch = new ColorMatch();

	private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
	private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
	private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

	private Color targetColor;

	ShuffleboardTab wheelTab = Shuffleboard.getTab("Wheel");

	/**
	 * Creates a new WheelSubsystem.
	 * 
	 * @param motor      - the wheel motor
	 * 
	 * @param encoder    - the wheel encoder
	 * 
	 * @param colorV3    - the color sensor
	 * 
	 * @param colorMatch - color match
	 * 
	 */
	public WheelSubsystem(WPI_TalonFX motor, ColorSensorV3 colorV3) {
		m_motor = motor;
		m_colorV3 = colorV3;

		m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_motor.setSelectedSensorPosition(0);
		m_motor.setNeutralMode(NeutralMode.Brake);

		m_colorMatch.addColorMatch(kBlueTarget);
		m_colorMatch.addColorMatch(kGreenTarget);
		m_colorMatch.addColorMatch(kRedTarget);
		m_colorMatch.addColorMatch(kYellowTarget);
	}

	/**
	 * gets the target color letter and returns the color the robot will see at
	 * target color
	 */
	public String targetColor() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			switch (gameData.charAt(0)) {
			case 'B':
				// the robot will see red at this point
				// so spin until it sees red
				targetColor = kRedTarget;
				return "Red";

			case 'G':
				// the robot will see yellow at this point
				// so spin until it sees yellow
				targetColor = kYellowTarget;
				return "Yellow";

			case 'R':
				// the robot will see blue at this point
				// so spin until it sees blue
				targetColor = kBlueTarget;
				return "Blue";

			case 'Y':
				// the robot will see green at this point
				// so spin until it sees green
				targetColor = kGreenTarget;
				return "Green";

			default:
				// This is corrupt data
				targetColor = null;
				break;
			}
		} else {
			// Code for no data received yet
			return null;
		}
		return null;
	}

	/**
	 * Gets the current count of wheel
	 */
	public double getRotations() {
		return m_motor.getSelectedSensorPosition(0) / 2048 * (Math.PI * Units.inchesToMeters(4));
	}

	/**
	 * Get the wheel velocity
	 * 
	 * @return The wheel's velocity
	 */
	public double getRate() {
		return m_motor.getSelectedSensorVelocity(0) * Math.PI * 8 * 10;
	}

	/**
	 * 
	 * @return
	 */
	public boolean hasRotated() {
		return (m_motor.getSelectedSensorPosition(0) / 2048
				* (Math.PI * Units.inchesToMeters(4))) >= Constants.WheelConstants.ROTATIONS;
	}

	/**
	 * Gets the closest color from the current color to the colors shown on the
	 * control panel.
	 */
	public String getClosestColor() {
		Color detectedColor = m_colorV3.getColor();
		String colorString;
		ColorMatchResult match = m_colorMatch.matchClosestColor(detectedColor);

		if (match.color == kBlueTarget) {
			colorString = "Blue";
		} else if (match.color == kRedTarget) {
			colorString = "Red";
		} else if (match.color == kGreenTarget) {
			colorString = "Green";
		} else if (match.color == kYellowTarget) {
			colorString = "Yellow";
		} else {
			colorString = "Unknown";
		}

		wheelTab.add("Red", detectedColor.red);
		wheelTab.add("Green", detectedColor.green);
		wheelTab.add("Blue", detectedColor.blue);
		wheelTab.add("Green", detectedColor.green);

		return colorString;
	}

	/**
	 * @param speed : Speed that the wheel motor moves
	 */
	public void rotateSpeed(double speed) {
		m_motor.set(speed);
	}

	/**
	 * Stops the wheel motor
	 */
	public void turnOffMotor() {
		m_motor.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		wheelTab.add("Detected Color", getClosestColor());
	}
}
