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

public class ClimbSubsystem extends SubsystemBase {
	private WPI_TalonFX m_winchFx, m_riserFx;

	/**
	 * The subsystem that manipulates the climb
	 * 
	 * @param winchFx - motor to drive the winch
	 * @param riseFx  - motor to raise hook
	 */
	public ClimbSubsystem(WPI_TalonFX winchFx, WPI_TalonFX riserFx) {
		m_winchFx = winchFx;
		m_riserFx = riserFx;
		m_winchFx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
		m_winchFx.setSelectedSensorPosition(0);
	}

	/**
	 * Moves the winch motor
	 * 
	 * @param speed - the speed for the motor to move
	 */
	public void driveWinch(double speed) {
		m_winchFx.set(speed);
	}

	/**
	 * Stops the winch motor
	 */
	public void stopWinch() {
		m_winchFx.stopMotor();
	}

	/**
	 * Moves the hook motor
	 * 
	 * @param speed - the speed for the hook to move
	 */
	public void driveHook(double speed) {
		m_riserFx.set(speed);
	}

	/**
	 * Stops the hook motor
	 */
	public void stopHook() {
		m_riserFx.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
