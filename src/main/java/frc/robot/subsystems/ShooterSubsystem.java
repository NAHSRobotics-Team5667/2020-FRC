/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState.States;

public class ShooterSubsystem extends PIDSubsystem {

	private WPI_TalonFX m_slaveWheel, m_masterWheel;
	private Rev2mDistanceSensor m_shooterSensor = new Rev2mDistanceSensor(Port.kMXP, Unit.kInches, RangeProfile.kHighAccuracy);

	private boolean previousSeenBallExit = false;


	/**
	 * Creates a shooter subsystem
	 * 
	 * @param slaveWheel  - motor controller that follows
	 * @param masterWheel - motor controller that controls the slave wheel
	 */

	public ShooterSubsystem(WPI_TalonFX slaveWheel, WPI_TalonFX masterWheel) {
		super(new PIDController(Constants.ShooterConstants.kP, 0, Constants.ShooterConstants.kD));
		m_slaveWheel = slaveWheel;
		m_masterWheel = masterWheel;
		m_masterWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_masterWheel.setSelectedSensorPosition(0);
		m_slaveWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_slaveWheel.setSelectedSensorPosition(0);
		m_slaveWheel.follow(m_masterWheel);
	}

	/**
	 * Calculates the speed of the wheels needed
	 */
	public void calculateSpeed() {

	}

	/**
	 * Fires the shooting wheels
	 * 
	 * @param speed - the speed of the wheels thats needed
	 */
	public void fire(double speed) {
		Constants.m_RobotState.setState(States.SHOOTING);
		m_masterWheel.set(speed);
	}

	public void setVoltage(double voltage){
		fire(voltage / 12);
	}

	/**
	 * Stops the shooter from firing
	 * 
	 */
	public void stopFire() {
		m_masterWheel.stopMotor();
	}

	/**
	 * Resets the encoder for the right shooter wheel
	 */
	public void resetEncoder() {
		m_masterWheel.setSelectedSensorPosition(0);
		m_slaveWheel.setSelectedSensorPosition(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public double getCurrentRPM() {
		return 0;
	}

	/**
	 * Determine if the Rev2mDistanceSensor saw a power cell leave the robot or not
	 * 
	 * @return true if ball is absent, therefore seen leaving robot
	 */
	public boolean hasSeenBallExit() {
		boolean currentSeenBallExit = false;
		if (m_shooterSensor.getRange(Unit.kInches) <= Constants.IntakeConstants.SENSOR_RANGE_INCHES
				&& previousSeenBallExit == true) {
			currentSeenBallExit = false;
			previousSeenBallExit = currentSeenBallExit;
		} else if (m_shooterSensor.getRange(Unit.kInches) > Constants.IntakeConstants.SENSOR_RANGE_INCHES
				&& previousSeenBallExit == false) {
			currentSeenBallExit = true;
			previousSeenBallExit = currentSeenBallExit;
		}
		return currentSeenBallExit;
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		setVoltage(Constants.ShooterConstants.ksVolts + output);
	}

	@Override
	protected double getMeasurement() {
		return getCurrentRPM();
	}

}
