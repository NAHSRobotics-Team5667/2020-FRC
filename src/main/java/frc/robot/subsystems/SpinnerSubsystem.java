/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class SpinnerSubsystem extends PIDSubsystem {
	private double m_setpoint;
	private WPI_TalonSRX motor;
	private Encoder encoder;

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
	public SpinnerSubsystem(WPI_TalonSRX motor, Encoder encoder, double kP, double kI, double kD) {
		super(new PIDController(kP, kI, kD));
		this.motor = motor;
		this.encoder = encoder;

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

	/**
	 * returns what the encoder is reading
	 */
	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return encoder.get();
	}
}
