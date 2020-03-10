/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState.States;

public class ClimbSubsystem extends SubsystemBase {
	private SpeedControllerGroup m_winchFx;
	private Solenoid m_delivery;

	private boolean m_deployed = false;
	private boolean m_status = false;

	/**
	 * The subsystem that manipulates the climb
	 * 
	 * @param winchFx - motor to drive the winch
	 * @param riseFx  - motor to raise hook
	 */
	public ClimbSubsystem(SpeedControllerGroup winchFx, Solenoid delivery) {
		m_winchFx = winchFx;
		m_delivery = delivery;
	}

	/**
	 * Moves the winch motor
	 * 
	 * @param speed - the speed for the motor to move
	 */
	public void driveWinch(double speed) {
		m_winchFx.set(speed);
		Constants.m_RobotState.setState(States.CLIMBING);
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
	public void deliverHook() {
		m_delivery.set(true);
		m_deployed = true;
		m_status = true;
	}

	/**
	 * Stops the hook motor
	 */
	public void lowerHook() {
		m_delivery.set(false);
		m_status = false;
	}

	public void toggle() {
		if (m_status == true)
			lowerHook();
		else
			deliverHook();
	}

	/**
	 * Check if the hook has been deployed
	 * 
	 * @return - Has the hook piston been extended
	 */
	public boolean hasDeployedHook() {
		return m_deployed;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
