/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

	private WPI_TalonSRX m_rightMaster, m_leftMaster, m_rightSlave, m_leftSlave;

	private SpeedControllerGroup m_left, m_right;

	private DifferentialDrive m_drive;

	private final AHRS m_gyro;

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
	 * Create a Drive Train subsystem
	 * 
	 * @param rightMaster - The right talon with encoder
	 * @param leftMaster  - The left talon with encoder
	 * @param rightSlave  - The right talon without encoder
	 * @param leftSlave   - The left talon without encoder
	 * @param gyro        - The gyro
	 */

	public DriveTrainSubsystem(WPI_TalonSRX rightMaster, WPI_TalonSRX leftMaster, WPI_TalonSRX rightSlave,
			WPI_TalonSRX leftSlave, AHRS gyro) {

		m_rightMaster = rightMaster;
		m_leftMaster = leftMaster;
		m_rightSlave = rightSlave;
		m_leftSlave = leftSlave;

		m_rightMaster.configFactoryDefault();
		m_leftMaster.configFactoryDefault();
		m_rightSlave.configFactoryDefault();
		m_leftSlave.configFactoryDefault();

		m_rightMaster.setSensorPhase(false);
		m_leftMaster.setSensorPhase(true);

		m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

		m_left = new SpeedControllerGroup(m_leftMaster, m_leftSlave);
		m_right = new SpeedControllerGroup(m_rightMaster, m_rightSlave);

		m_drive = new DifferentialDrive(m_left, m_right);

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
		m_odometry.update(Rotation2d.fromDegrees(getHeading()), -getLeftRotations(), getRightRotations());
		SmartDashboard.putNumber("Right Pos", getRightRotations());
		SmartDashboard.putNumber("Left Pos", getLeftRotations());
		SmartDashboard.putNumber("Right Vel", getRightEncoderRate());
		SmartDashboard.putNumber("Left Vel", getLeftEncoderRate());
		SmartDashboard.putNumber("Gyro", getHeading());
		SmartDashboard.putNumber("Gyro Vel X", m_gyro.getVelocityX());
		SmartDashboard.putNumber("Gyro Vel Y", m_gyro.getVelocityY());
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
		return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose - The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts  the commanded left output
	 * @param rightVolts the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_left.setVoltage(leftVolts);
		m_right.setVoltage(rightVolts);

		SmartDashboard.putNumber("l_volts", leftVolts);
		SmartDashboard.putNumber("r_volts", rightVolts);
	}

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void resetEncoders() {
		m_rightMaster.setSelectedSensorPosition(0);
		m_leftMaster.setSelectedSensorPosition(0);
	}

	/**
	 * Gets the average distance of the two encoders.
	 *
	 * @return the average of the two encoder readings
	 */
	public double getAverageEncoderDistance() {
		return (getLeftRotations() + getRightRotations()) / 2.0;
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

	/**
	 * Get the distance traveled on the right side
	 * 
	 * @return The distance traveled in meters from the right encoder
	 */
	public double getRightRotations() {
		return m_rightMaster.getSelectedSensorPosition(0) * Constants.DriveConstants.ENCODER_CONSTANT;
	}

	/**
	 * Get the distance traveled on the left side
	 * 
	 * @return The distance traveled in meters from the left encoder
	 */
	public double getLeftRotations() {
		return m_leftMaster.getSelectedSensorPosition(0) * Constants.DriveConstants.ENCODER_CONSTANT;
	}

	/**
	 * Drive using the curvatureDrive method
	 * 
	 * @param throttle  - Speed
	 * @param angle     - Rotation
	 * @param quickTurn - is quick turn
	 */
	public void curvatureDrive(double throttle, double angle, boolean quickTurn) {
		m_drive.curvatureDrive(throttle, angle, quickTurn);
	}

	/**
	 * Get the left encoder velocity
	 * 
	 * @return Get the left encoder velocity in m/s
	 */
	public double getLeftEncoderRate() {
		return m_leftMaster.getSelectedSensorVelocity(0) * Constants.DriveConstants.ENCODER_CONSTANT * 10;
	}

	/**
	 * Geth the right encoder velocity
	 * 
	 * @return Get the right encoder velocity in m/s
	 */
	public double getRightEncoderRate() {
		return m_leftMaster.getSelectedSensorVelocity(0) * Constants.DriveConstants.ENCODER_CONSTANT * 10;
	}

}
