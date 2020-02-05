/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState.States;

public class ShooterSubsystem extends SubsystemBase {

	private SpeedController m_rightWheel, m_leftWheel, m_angleWheel;
	private Encoder m_rightEncoder, m_leftEncoder, m_angleEncoder;

	/**
	 * Creates a shooter subsystem
	 * 
	 * @param rightWheel   - motor controller that controls the right shooter wheel
	 * @param leftWheel    - motor controller that controls the left shooter wheel
	 * @param angleWheel   - motor controller that controls the motor to manipulate
	 *                     the shooter's angle
	 * @param rightEncoder - encoder to track the progress of the right shooter
	 *                     wheel
	 * @param leftEncoder  - encoder to track the progress of the left shooter wheel
	 * @param angleEncoder - encoder to track the progress of the shooter arm
	 */

	public ShooterSubsystem(SpeedController rightWheel, SpeedController leftWheel, SpeedController angleWheel,
			Encoder rightEncoder, Encoder leftEncoder, Encoder angleEncoder) {
		m_rightWheel = rightWheel;
		m_leftWheel = leftWheel;
		m_angleWheel = angleWheel;
		m_rightEncoder = rightEncoder;
		m_leftEncoder = leftEncoder;
		m_angleEncoder = angleEncoder;
	}

	/**
	 * Calculates the speed of the shooting wheels
	 */
	public void calculateSpeeds() {
		// to calculate the different speeds of the right and left motors based on angle
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
	 * Causes the angle of the shooter to decrease
	 * 
	 * @param angleSpeed - speed of the motor that controls the angle of the shooter
	 */
	public void goDown(double angleSpeed) {
		m_angleWheel.set(-angleSpeed);
	}

	/**
	 * Causes the angle of the shooter to increase
	 * 
	 * @param angleSpeed - speed of the motor that controls the angle of the shooter
	 */
	public void goUp(double angleSpeed) {
		m_angleWheel.set(angleSpeed);
	}

	/**
	 * Stops the shooter arm from moving
	 */
	public void angleStop() {
		m_angleWheel.stopMotor();
	}

	/**
	 * Returns the current angle/position of the shooter
	 * 
	 * @return - the current angle/position of the shooter
	 */
	public int getCurrentAngle() {
		// covert experimental values to angles
		return (m_angleEncoder.get());
	}

	/**
	 * Resets the encoder for the right shooter wheel
	 */
	public void resetRightEncoder() {
		m_rightEncoder.reset();
	}

	/**
	 * Resets the encoder for the left shooter wheel
	 */
	public void resetLeftEncoder() {
		m_leftEncoder.reset();
	}

	/**
	 * Resets the encoder for the motor controlling the shooter's angle
	 */
	public void resetAngleEncoder() {
		m_angleEncoder.reset();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}