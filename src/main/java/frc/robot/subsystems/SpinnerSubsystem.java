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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class SpinnerSubsystem extends PIDSubsystem {
	private WPI_TalonFX motor;

	public double rotations = 2.7 * 3;

	private ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
	private final ColorMatch m_colorMatcher = new ColorMatch();

	private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
	private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
	private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

	/**
	 * creates new PID Controller
	 * 
	 * @param motor   - WPITalonSRX motor to spin wheel
	 * @param encoder - measures motor rotation
	 * 
	 * @param kP      - kP
	 * @param kI      - kI
	 * @param kD      - kD
	 */
	public SpinnerSubsystem(WPI_TalonFX motor, double kP, double kI, double kD) {
		super(new PIDController(kP, kI, kD));
		this.motor = motor;

		this.motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		this.motor.setNeutralMode(NeutralMode.Brake);
		reset();

		m_colorMatcher.addColorMatch(kBlueTarget);
		m_colorMatcher.addColorMatch(kGreenTarget);
		m_colorMatcher.addColorMatch(kRedTarget);
		m_colorMatcher.addColorMatch(kYellowTarget);
	}

	public void drive(double throttle) {
		motor.set(throttle);

	}

	public boolean hasRotated() {
		return (motor.getSelectedSensorPosition(0) / 2048 * (Math.PI * Units.inchesToMeters(4))) >= rotations;
	}

	public void reset() {
		motor.setSelectedSensorPosition(0);
	}

	/**
	 * uses output from encoder and compares to the setpoint
	 * 
	 * @param output   - the output from PID controller
	 * @param setpoint - the setpoint
	 */

	@Override
	public void useOutput(final double output, final double setpoint) {
		motor.set(output);
	}

	public void position() {
		super.setSetpoint(0);
	}

	public void control() {
		super.setSetpoint(rotations);
	}

	/***
	 * 
	 * @return the name of the detected color as a string
	 */
	public String detectColor() {
		Color detectedColor = m_colorSensor.getColor();
		String colorString;
		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

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
		SmartDashboard.putNumber("Red", detectedColor.red);
		SmartDashboard.putNumber("Green", detectedColor.green);
		SmartDashboard.putNumber("Blue", detectedColor.blue);
		SmartDashboard.putNumber("Confidence", match.confidence);
		SmartDashboard.putString("Detected Color", colorString);

		return colorString;
	}

	/**
	 * returns what the encoder is reading
	 */
	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return motor.getSelectedSensorPosition(0) / 2048 * (Math.PI * Units.inchesToMeters(4));
	}

	@Override
	public void periodic() {
		super.periodic();
		SmartDashboard.putNumber("Rotations",
				motor.getSelectedSensorPosition(0) / 2048 * (Math.PI * Units.inchesToMeters(4)));
		detectColor();

	}

	public double getRate() {
		return motor.getSelectedSensorVelocity(0) * Math.PI * 8 * 10;
	}
}
