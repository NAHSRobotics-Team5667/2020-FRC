/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState.States;
import frc.robot.sensors.Rev2mTOF;

public class ShooterSubsystem extends SubsystemBase {

	private WPI_TalonFX m_master;
	private ShuffleboardTab compTab = Shuffleboard.getTab("Teleop");

	private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.ShooterConstants.ksVolts,
			Constants.ShooterConstants.kvVoltSecondsPerMeter, Constants.ShooterConstants.kaVoltSecondsSquaredPerMeter);

	private PIDController m_controller = new PIDController(Constants.ShooterConstants.kP, Constants.ShooterConstants.kI,
			Constants.ShooterConstants.kD);

	/**
	 * Creates a shooter subsystem
	 * 
	 * @param masterWheel - motor controller that controls the slave wheel
	 */

	public ShooterSubsystem(WPI_TalonFX master) {

		m_master = master;
		m_master.configFactoryDefault();
		m_master.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		configuration.openloopRamp = 2;
		m_master.setSelectedSensorPosition(0);

		setNeutralMode(NeutralMode.Coast);

		outputTelemetry();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler
	}

	public void setNeutralMode(NeutralMode mode) {
		m_master.setNeutralMode(mode);
	}

	/**
	 * Fires the shooting wheels
	 * 
	 * @param speed - the speed of the wheels thats needed
	 */
	public void fire(double speed) {
		Constants.m_RobotState.setState(States.SHOOTING);
		m_master.set(-speed);
	}

	public void fireRPM(double desiredRPM) {
		m_controller.setSetpoint(desiredRPM);
		double ff = Constants.ShooterConstants.ksVolts + desiredRPM * Constants.ShooterConstants.kvVoltSecondsPerMeter;
		setVoltage(m_controller.calculate(getCurrentRPM()) + ff);
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
	}

	/**
	 * Get the current RPM of the shooter wheels
	 * 
	 * @return - The current RPM of the shooter wheels
	 */
	public double getCurrentRPM() {
		return -m_master.getSelectedSensorVelocity(0) * Constants.ShooterConstants.ENCODER_CONSTANT * 600.0;
	}

	public void resetIError() {
		m_controller.reset();
	}

	/**
	 * Output the shooter's telemetry
	 */
	public void outputTelemetry() {
		compTab.addNumber("Shooter RPM", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getCurrentRPM();
			}
		}).withWidget(BuiltInWidgets.kGraph);

	}
}
