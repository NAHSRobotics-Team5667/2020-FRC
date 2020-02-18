/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
	private WPI_TalonFX m_winchFx, m_riserFx;

	/**
	 * The subsystem that manipulates the climb
	 * 
	 * @param motor
	 */
	public ClimbSubsystem(WPI_TalonFX winchFx, WPI_TalonFX riserFx) {
		m_winchFx = winchFx;
		m_riserFx = riserFx;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
