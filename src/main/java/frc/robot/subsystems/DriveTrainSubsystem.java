/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {

	private SpeedController m_fRight;
	private SpeedController m_fLeft;
	private SpeedController m_bRight;
	private SpeedController m_bLeft;

	private SpeedControllerGroup m_left;
	private SpeedControllerGroup m_right;

	private Encoder m_lEncoder;
	private Encoder m_rEncoder;

	private DifferentialDrive m_drive;

	private DriveModes m_driveMode = DriveModes.MANUAL;

	public static enum DriveModes {
		MANUAL(0), AUTO(1);

		private int mode;

		private DriveModes(int mode) {
			this.mode = mode;
		}

		public int getMode() {
			return mode;
		}
	}

	/**
	 * Drive Train Subsystem
	 * 
	 * @param fRight   - The Front Right Motor
	 * @param fLeft    - The Front Left Motor
	 * @param bRight   - The Back Right Motor
	 * @param bLeft    - The Back Left Motor
	 * @param lEncoder - The Left Encoder
	 * @param rEncoder - The Right Encoder
	 */
	public DriveTrainSubsystem(SpeedController fRight, SpeedController fLeft, SpeedController bRight,
			SpeedController bLeft, Encoder lEncoder, Encoder rEncoder) {

		m_fRight = fRight;
		m_fLeft = fLeft;
		m_bRight = bRight;
		m_bLeft = bLeft;

		m_left = new SpeedControllerGroup(m_fLeft, m_bLeft);
		m_right = new SpeedControllerGroup(m_fRight, m_bRight);

		m_drive = new DifferentialDrive(m_left, m_right);

		m_lEncoder = lEncoder;
		m_rEncoder = rEncoder;
	}

	/**
	 * Drive the robot using arcade control
	 * 
	 * @param throttle - How fast it should drive (-1, 1)
	 * @param angle    - Change in heading
	 */
	public void drive(double throttle, double angle) {
		m_drive.arcadeDrive(throttle, angle);
	}

	/**
	 * Get the right encoder distance
	 * 
	 * @return the right encoder distance
	 */
	public double getRightEncoder() {
		return m_rEncoder.getDistance();
	}

	/**
	 * Get the left encoder distance
	 * 
	 * @return the left encoder distance
	 */
	public double getLeftEncoder() {
		return m_lEncoder.getDistance();
	}

	/**
	 * Get the right raw encoder values
	 * 
	 * @return - the right raw encoder values
	 */
	public int getRightEncoderRaw() {
		return m_rEncoder.get();
	}

	/**
	 * Get the left raw encoder values
	 * 
	 * @return - The left raw encoder values
	 */
	public int getLeftEncoderRaw() {
		return m_lEncoder.get();
	}

	/**
	 * Get the current drive mode
	 * 
	 * @return - The current drive mode
	 */
	public DriveModes getDriveMode() {
		return m_driveMode;
	}

	/**
	 * Set the current drive mode
	 * 
	 * @param mode - The current drive train mode
	 */
	public void setDriveMode(DriveModes mode) {
		m_driveMode = mode;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
