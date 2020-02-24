/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class that manipulates the Wheel of Fortune
 */
public class WheelSubsystem extends SubsystemBase {
	private WPI_TalonFX m_motor;
	private ColorSensorV3 m_colorV3 = new ColorSensorV3(Port.kOnboard);
	private ColorMatch m_colorMatch = new ColorMatch();

	private ShuffleboardTab compTab = Shuffleboard.getTab("Teleop");

	private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
	private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
	private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

	/**
	 * A Subsystem that manipulates the wheel of fortune
	 * 
	 * @param motor - The motor spinning the wheel
	 */
	public WheelSubsystem(WPI_TalonFX motor) {
		m_motor = motor;

		// Reset the motor to default configuration
		m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_motor.setSelectedSensorPosition(0);
		m_motor.setNeutralMode(NeutralMode.Brake);
		// Set the colors
		m_colorMatch.addColorMatch(kBlueTarget);
		m_colorMatch.addColorMatch(kGreenTarget);
		m_colorMatch.addColorMatch(kRedTarget);
		m_colorMatch.addColorMatch(kYellowTarget);

		outputTelemetry();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/**
	 * Get the target color based on the FMS data
	 * 
	 * @return - The target color as a string
	 */
	public String targetColor() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			switch (gameData.charAt(0)) {
			case 'B':
				// the robot will see red at this point
				// so spin until it sees red
				return "Red";

			case 'G':
				// the robot will see yellow at this point
				// so spin until it sees yellow
				return "Yellow";

			case 'R':
				// the robot will see blue at this point
				// so spin until it sees blue
				return "Blue";

			case 'Y':
				// the robot will see green at this point
				// so spin until it sees green
				return "Green";

			default:
				// This is corrupt data
				break;
			}
		} else {
			// Code for no data received yet
			return "null";
		}
		return "null";
	}

	/**
	 * Get the current amount of rotations completed
	 * 
	 * @return - The amount of rotations the wheel has completed
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
	 * Has the wheel rotated the amount of times desired
	 * 
	 * @return - boolean (have we rotated enough times)
	 */
	public boolean hasRotated() {
		return (m_motor.getSelectedSensorPosition(0) / 2048
				* (Math.PI * Units.inchesToMeters(4))) >= Constants.WheelConstants.ROTATIONS;
	}

	/**
	 * Get the closest detected color
	 * 
	 * @return - The closest color detected by the color sensor
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

		return colorString;
	}

	/**
	 * Rotate the wheel at a set speed
	 * 
	 * @param speed - The desired wheel speed
	 */
	public void rotateSpeed(double speed) {
		m_motor.set(speed);
	}

	/**
	 * Turn the motor off
	 */
	public void turnOffMotor() {
		m_motor.stopMotor();
	}

	public void resetEncoder() {
		m_motor.setSelectedSensorPosition(0);
	}

	/**
	 * Output telemetry function
	 */
	public void outputTelemetry() {
		compTab.addString("Target Color", new Supplier<String>() {
			@Override
			public String get() {
				return targetColor();
			}
		});

		compTab.addString("Current Color", new Supplier<String>() {
			@Override
			public String get() {
				return getClosestColor();
			}
		});
		compTab.addNumber("Rotations", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getRotations();
			}
		});
	}
}
