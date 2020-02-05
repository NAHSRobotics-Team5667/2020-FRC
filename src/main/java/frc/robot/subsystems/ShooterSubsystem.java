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

public class ShooterSubsystem extends SubsystemBase {

	private SpeedController m_rightWheel, m_leftWheel;
	private Encoder m_rightEncoder, m_leftEncoder;

	/**
	 * Creates a shooter subsystem
	 * 
	 * @param rightWheel   - motor controller that controls the right shooter wheel
	 * @param leftWheel    - motor controller that controls the left shooter wheel
	 * 
	 * @param rightEncoder - encoder to track the progress of the right shooter
	 *                     wheel
	 * @param leftEncoder  - encoder to track the progress of the left shooter wheel
	 */

	public ShooterSubsystem(SpeedController rightWheel, SpeedController leftWheel, Encoder rightEncoder,
			Encoder leftEncoder) {
		m_rightWheel = rightWheel;
		m_leftWheel = leftWheel;
		m_rightEncoder = rightEncoder;
		m_leftEncoder = leftEncoder;
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
		m_rightEncoder.reset();
	}

	/**
	 * Resets the encoder for the left shooter wheel
	 */
	public void resetLeftEncoder() {
		m_leftEncoder.reset();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}