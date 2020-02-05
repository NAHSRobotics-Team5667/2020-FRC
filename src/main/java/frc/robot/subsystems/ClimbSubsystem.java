/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ClimbSubsystem extends PIDSubsystem {
	private double m_setpoint;
	private WPI_TalonFX m_motor;

	/**
	 * Creates new ClimbSubsystem
	 * 
	 * @param motor   - WPI_TalonFX to power the climbing system
	 * @param encoder - encoder to track the progress of the motor
	 * 
	 * @param kP      - kP value
	 * @param kI      - kI value
	 * @param kD      - kD value
	 */
	public ClimbSubsystem(WPI_TalonFX motor, double kP, double kI, double kD) {
		super(new PIDController(kP, kI, kD));
		this.m_motor = motor;
	}

	/**
	 * Powers motor until climb is reached
	 */
	public void climb() {
		super.setSetpoint(m_setpoint);
	}

	/**
	 * Uses output from the encoder and compares it to the setpoint
	 * 
	 * @param output   - the output from the PID controller
	 * @param setpoint - the specified setpoint
	 */
	public void useOutput(final double output, final double setpoint) {
		m_motor.set(output);
	}

	/**
	 * Resets the value of the encoder
	 */
	public void resetEncoder() {
		m_motor.setSelectedSensorPosition(0);
	}

	/**
	 * Returns the value from the encoder
	 */
	protected double getMeasurement() {
		return m_motor.getSelectedSensorPosition(0) * Constants.ClimbConstants.ENCODER_CONSTANT;
	}

}