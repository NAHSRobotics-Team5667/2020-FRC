/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

	private WPI_TalonFX m_rightMaster, m_leftMaster, m_rightSlave, m_leftSlave;

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
	 * @param rightMaster - The right falcon with encoder
	 * @param leftMaster  - The left falcon with encoder
	 * @param rightSlave  - The right falcon without encoder
	 * @param leftSlave   - The left falcon without encoder
	 * @param gyro        - The gyro
	 */
	public DriveTrainSubsystem(WPI_TalonFX rightMaster, WPI_TalonFX leftMaster, WPI_TalonFX rightSlave,
			WPI_TalonFX leftSlave, AHRS gyro) {

		m_rightMaster = rightMaster;
		m_leftMaster = leftMaster;
		m_rightSlave = rightSlave;
		m_leftSlave = leftSlave;

		TalonFXConfiguration falconConfig = new TalonFXConfiguration();
		falconConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		falconConfig.neutralDeadband = Constants.DriveConstants.DEADBAND;
		falconConfig.slot0.kP = Constants.DriveConstants.kP;
		falconConfig.slot0.kI = 0.0;
		falconConfig.slot0.kD = Constants.DriveConstants.kD;
		falconConfig.slot0.integralZone = 400;
		falconConfig.slot0.closedLoopPeakOutput = 1.0;
		falconConfig.closedloopRamp = Constants.DriveConstants.CLOSED_LOOP_RAMP;
		falconConfig.openloopRamp = Constants.DriveConstants.OPEN_LOOP_RAMP;

		m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

		setNeutralMode(NeutralMode.Brake);

		m_rightMaster.setSensorPhase(false);
		m_leftMaster.setSensorPhase(true);

		m_rightMaster.overrideLimitSwitchesEnable(false);
		m_leftMaster.overrideLimitSwitchesEnable(false);

		m_rightMaster.configAllSettings(falconConfig);
		m_leftMaster.configAllSettings(falconConfig);

		m_leftSlave.follow(m_leftMaster);
		m_rightSlave.follow(m_rightMaster);

		m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

		m_gyro = gyro;

		resetOdometry(new Pose2d());
		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(Rotation2d.fromDegrees(getHeading()), stepsToMeters(getLeftEncoderPosition()),
				stepsToMeters(getRightEncoderPosition()));
		SmartDashboard.putString("Pose", m_odometry.getPoseMeters().toString());
		SmartDashboard.putNumber("Right Position", stepsToMeters(getRightEncoderPosition()));
		SmartDashboard.putNumber("Left Position", stepsToMeters(getLeftEncoderPosition()));

		SmartDashboard.putNumber("Right Vel", getRightEncoderRate());
		SmartDashboard.putNumber("Left Vel", getLeftEncoderRate());

	}

	/**
	 * Sets the neutral mode for the drive train
	 * 
	 * @param neutralMode the desired neutral mode
	 */
	public void setNeutralMode(NeutralMode neutralMode) {
		m_leftMaster.setNeutralMode(neutralMode);
		m_leftSlave.setNeutralMode(neutralMode);
		m_rightMaster.setNeutralMode(neutralMode);
		m_rightSlave.setNeutralMode(neutralMode);
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
		zeroDriveTrainEncoders();
		m_gyro.zeroYaw();
		m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts  the commanded left output
	 * @param rightVolts the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		double l = leftVolts;
		double r = -rightVolts;

		m_leftMaster.setVoltage(l);
		m_leftMaster.setVoltage(r);

		SmartDashboard.putNumber("l_volts", l);
		SmartDashboard.putNumber("r_volts", r);
	}

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	public void zeroDriveTrainEncoders() {
		m_rightMaster.setSelectedSensorPosition(0);
		m_leftMaster.setSelectedSensorPosition(0);
	}

	/**
	 * Gets the average distance of the two encoders.
	 *
	 * @return the average of the two encoder readings
	 */
	public double getAverageEncoderDistance() {
		return (stepsToMeters(getLeftEncoderPosition()) + stepsToMeters(getRightEncoderPosition())) / 2.0;
	}

	/**
	 * Zeroes the heading of the robot.
	 */
	public void zeroHeading() {
		m_gyro.reset();
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
	 * returns left encoder position
	 * 
	 * @return left encoder position
	 */
	public int getLeftEncoderPosition() {
		return m_leftMaster.getSelectedSensorPosition(0);
	}

	/**
	 * Returns right encoder position
	 * 
	 * @return right encoder position
	 */
	public int getRightEncoderPosition() {
		return m_rightMaster.getSelectedSensorPosition(0);
	}

	/**
	 * Get the left encoder velocity
	 * 
	 * @return Get the left encoder velocity in m/s
	 */
	public double getLeftEncoderRate() {
		return m_leftMaster.getSelectedSensorVelocity(0) * Constants.DriveConstants.encoderConstant * 10;
	}

	/**
	 * Geth the right encoder velocity
	 * 
	 * @return Get the right encoder velocity in m/s
	 */
	public double getRightEncoderRate() {
		return m_rightMaster.getSelectedSensorVelocity(0) * Constants.DriveConstants.encoderConstant * 10;
	}

	/**
	 * Returns the heading of the robot in form required for odometry.
	 *
	 * @return the robot's heading in degrees, from 180 to 180 with positive value
	 *         for left turn.
	 */
	public double getHeading() {
		return Math.IEEEremainder(m_gyro.getAngle(), 360.0d) * -1.0d;
	}

	/**
	 * Converts from encoder steps to meters.
	 * 
	 * @param steps encoder steps to convert
	 * @return meters
	 */
	public static double stepsToMeters(int steps) {
		return (Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS
				/ Constants.DriveConstants.SENSOR_UNITS_PER_ROTATION) * steps;
	}

	/**
	 * Converts from encoder units per 100 milliseconds to meters per second.
	 * 
	 * @param stepsPerDecisec steps per decisecond
	 * @return meters per second
	 */
	public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
		return stepsToMeters(stepsPerDecisec * 10);
	}

	/**
	 * Converts from meters to encoder units.
	 * 
	 * @param meters meters
	 * @return encoder units
	 */
	public static double metersToSteps(double meters) {
		return (meters / Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS)
				* Constants.DriveConstants.SENSOR_UNITS_PER_ROTATION;
	}

	/**
	 * Convers from meters per second to encoder units per 100 milliseconds.
	 * 
	 * @param metersPerSec meters per second
	 * @return encoder units per decisecond
	 */
	public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
		return metersToSteps(metersPerSec) * .1d;
	}

}
