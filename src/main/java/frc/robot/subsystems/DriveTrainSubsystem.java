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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

	private final Gyro m_gyro;

	private final DifferentialDriveOdometry m_odometry;

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
			SpeedController bLeft, Encoder lEncoder, Encoder rEncoder, Gyro gyro) {

		m_fRight = fRight;
		m_fLeft = fLeft;
		m_bRight = bRight;
		m_bLeft = bLeft;

		m_left = new SpeedControllerGroup(m_fLeft, m_bLeft);
		m_right = new SpeedControllerGroup(m_fRight, m_bRight);

		m_drive = new DifferentialDrive(m_left, m_right);

		m_lEncoder = lEncoder;
		m_rEncoder = rEncoder;

		m_lEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
		m_rEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

		m_gyro = gyro;

		resetEncoders();
		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
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

	/**
	 * Stop all motors
	 */
	public void stop() {
		m_drive.stopMotor();
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_lEncoder.getDistance(), m_rEncoder.getDistance());
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Returns the current wheel speeds of the robot.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_lEncoder.getRate(), m_rEncoder.getRate());
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
	}

	/**
	 * Drives the robot using arcade controls.
	 *
	 * @param fwd the commanded forward movement
	 * @param rot the commanded rotation
	 */
	public void arcadeDrive(double fwd, double rot) {
		m_drive.arcadeDrive(fwd, rot);
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts  the commanded left output
	 * @param rightVolts the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_left.setVoltage(leftVolts);
		m_right.setVoltage(-rightVolts);
	}

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void resetEncoders() {
		m_lEncoder.reset();
		m_rEncoder.reset();
	}

	/**
	 * Gets the average distance of the two encoders.
	 *
	 * @return the average of the two encoder readings
	 */
	public double getAverageEncoderDistance() {
		return (m_lEncoder.getDistance() + m_rEncoder.getDistance()) / 2.0;
	}

	/**
	 * Gets the left drive encoder.
	 *
	 * @return the left drive encoder
	 */
	public Encoder getLeftEncoder() {
		return m_lEncoder;
	}

	/**
	 * Gets the right drive encoder.
	 *
	 * @return the right drive encoder
	 */
	public Encoder getRightEncoder() {
		return m_rEncoder;
	}

	/**
	 *
	 * @param maxOutput the maximum output to which the drive will be constrained
	 */
	public void setMaxOutput(double maxOutput) {
		m_drive.setMaxOutput(maxOutput);
	}

	/**
	 * Zeroes the heading of the robot.
	 */
	public void zeroHeading() {
		m_gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from 180 to 180
	 */
	public double getHeading() {
		return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getRate() * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

}
