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
  private WPI_TalonFX m_motor;
    
  /**
   * Creates a new ClimbSubsystem.
   */
  public ClimbSubsystem(WPI_TalonFX motor) {
    m_motor = motor;
  }

  public double getClimbSpeed(){
    return m_motor.get();
  }

  public void climb(double speed){
      m_motor.set(speed);
  }
  public void stop(){
    m_motor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
