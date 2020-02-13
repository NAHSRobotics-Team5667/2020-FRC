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

	public void DriveWinch(double speed) {
		m_winchFx.set(speed);
	}

	public void StopWinch() {
		m_winchFx.stopMotor();
	}

	public void DriveHook(double speed) {
		m_riserFx.set(speed);
	}

	public void StopHook() {
		m_riserFx.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
