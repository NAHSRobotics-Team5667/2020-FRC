/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class SpinnerSubsystem extends PIDSubsystem {
	private WPI_TalonFX motor;
	private Encoder encoder;

	private ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.VisionConstants.COLOR_SENSOR_PORT);

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

	}

	public void drive(double throttle) {
		return motor.getSelectedSensorPosition(0) / motor.getSelectedSensorPosition(0);

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
		super.setSetpoint(0);
	}

	/***
	 * 
	 * @return the name of the detected color as a string
	 */
	public String detectColor() {
		String color = m_colorSensor.getColor().toString();
		SmartDashboard.putString("Color", color);
		return color;

	}

	/**
	 * returns what the encoder is reading
	 */
	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return encoder.get();
	}

	@Override
	public void periodic() {
		super.periodic();
		SmartDashboard.putNumber("Rotations",
				motor.getSelectedSensorPosition(0) / 2048 * (Math.PI * Units.inchesToMeters(4)));
	}

	public double getRate(){
		return motor.getSelectedSensorPosition(0)
	}
}
