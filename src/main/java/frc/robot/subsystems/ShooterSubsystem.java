/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState.States;

public class ShooterSubsystem extends SubsystemBase {

	private WPI_TalonFX m_rightWheel, m_leftWheel;

	/**
	 * Creates a shooter subsystem
	 * 
	 * @param rightWheel - motor controller that controls the right shooter wheel
	 * @param leftWheel  - motor controller that controls the left shooter wheel
	 */

	public ShooterSubsystem(WPI_TalonFX rightWheel, WPI_TalonFX leftWheel) {
		m_rightWheel = rightWheel;
		m_leftWheel = leftWheel;
		m_rightWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_rightWheel.setSelectedSensorPosition(0);
		m_leftWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_leftWheel.setSelectedSensorPosition(0);
	}

	/**
	 * Calculates the speed of the right shooting wheel
	 */
	public void calculateRightSpeed() {

	}

	/**
	 * Calculates the speed of the left shooting wheel
	 */
	public void calculateLeftSpeed() {

	}

	/**
	 * Fires the shooting wheels
	 * 
	 * @param rightSpeed - the speed of the right shooting wheel
	 * @param leftSpeed  - the speed of the left shooting wheel
	 */
	public void fire(double rightSpeed, double leftSpeed) {
		Constants.m_RobotState.setState(States.SHOOTING);
		m_rightWheel.set(rightSpeed);
		m_leftWheel.set(leftSpeed);
	}

	/**
	 * Stops the shooter from firing
	 * 
	 */
	public void stopFire() {
		m_rightWheel.stopMotor();
		m_leftWheel.stopMotor();
	}

	/**
	 * Resets the encoder for the right shooter wheel
	 */
	public void resetRightEncoder() {
		m_rightWheel.setSelectedSensorPosition(0);
	}

	/**
	 * Resets the encoder for the left shooter wheel
	 */
	public void resetLeftEncoder() {
		m_leftWheel.setSelectedSensorPosition(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}
