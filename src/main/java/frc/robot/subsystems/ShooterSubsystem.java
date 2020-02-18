/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotState.States;
import frc.robot.sensors.Rev2mTOF;

public class ShooterSubsystem extends PIDSubsystem {

	private WPI_TalonFX m_slave, m_master;
	public Rev2mTOF tof_sensor = new Rev2mTOF(Port.kMXP, Unit.kInches, RangeProfile.kHighAccuracy,
			Constants.IntakeConstants.SENSOR_RANGE_INCHES);

	private ShuffleboardTab compTab = Shuffleboard.getTab("Teleop");
	private ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");

	/**
	 * Creates a shooter subsystem
	 * 
	 * @param slaveWheel  - motor controller that follows
	 * @param masterWheel - motor controller that controls the slave wheel
	 */

	public ShooterSubsystem(WPI_TalonFX slave, WPI_TalonFX master) {
		super(new PIDController(Constants.ShooterConstants.kP, 0, Constants.ShooterConstants.kD));

		m_slave = slave;
		m_master = master;

		m_master.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_master.setSelectedSensorPosition(0);

		m_slave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_slave.setSelectedSensorPosition(0);

		m_slave.follow(m_master);

		setNeutralMode(NeutralMode.Coast);

		tof_sensor.enable();

		outputTelemetry();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void setNeutralMode(NeutralMode mode) {
		m_master.setNeutralMode(mode);
		m_slave.setNeutralMode(mode);
	}

	/**
	 * Fires the shooting wheels
	 * 
	 * @param speed - the speed of the wheels thats needed
	 */
	public void fire(double speed) {
		Constants.m_RobotState.setState(States.SHOOTING);
		m_master.set(speed);
	}

	/**
	 * Fire the shooting wheels using specific voltage
	 * 
	 * @param voltage - The voltage to set the wheels at
	 */
	public void setVoltage(double voltage) {
		fire(voltage / 12);
	}

	/**
	 * Stops the shooter from firing
	 */
	public void stopFire() {
		m_master.stopMotor();
	}

	/**
	 * Resets the encoder for the right shooter wheel
	 */
	public void resetEncoder() {
		m_master.setSelectedSensorPosition(0);
		m_slave.setSelectedSensorPosition(0);
	}

	/**
	 * Get the current RPM of the shooter wheels
	 * 
	 * @return - The current RPM of the shooter wheels
	 */
	public double getCurrentRPM() {
		return m_master.getSelectedSensorVelocity(0) * Constants.ShooterConstants.ENCODER_CONSTANT * 600;
	}

	/**
	 * Output the shooter's telemetry
	 */
	public void outputTelemetry() {
		compTab.add("Shooter RPM", getCurrentRPM()).withWidget(BuiltInWidgets.kGraph);
		autoTab.add("Shooter RPM", getCurrentRPM()).withWidget(BuiltInWidgets.kGraph);

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
