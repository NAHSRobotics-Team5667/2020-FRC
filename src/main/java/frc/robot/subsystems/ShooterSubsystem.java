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

	private WPI_TalonFX m_slaveWheel, m_masterWheel;

	/**
	 * Creates a shooter subsystem
	 * 
	 * @param slaveWheel  - motor controller that follows
	 * @param masterWheel - motor controller that controls the slave wheel
	 */

	public ShooterSubsystem(WPI_TalonFX slaveWheel, WPI_TalonFX masterWheel) {
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

}
