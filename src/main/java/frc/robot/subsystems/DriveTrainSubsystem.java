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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

  private SpeedController m_fRight, m_fLeft, m_bRight, m_bLeft;

  private SpeedControllerGroup m_right, m_left;

  private DifferentialDrive drive;

  private Encoder m_rightEncoder, m_leftEncoder;

  public DriveTrainSubsystem(SpeedController fRight, SpeedController fLeft, SpeedController bRight,
      SpeedController bLeft) {
    m_fRight = fRight;
    m_fLeft = fLeft;
    m_bRight = bRight;
    m_bLeft = bLeft;

    m_right = new SpeedControllerGroup(m_fRight, m_bRight);
    m_left = new SpeedControllerGroup(m_fLeft, m_bLeft);

    drive = new DifferentialDrive(m_left, m_right);

    m_rightEncoder = new Encoder(Constants.DriveTrainConstants.rightEncoderChannelA,
        Constants.DriveTrainConstants.rightEncoderChannelB);
    m_leftEncoder = new Encoder(Constants.DriveTrainConstants.leftEncoderChannnelA,
        Constants.DriveTrainConstants.leftEncoderChannelB);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double throttle, double rotation) {
    drive.arcadeDrive(throttle, rotation);
  }

  public void tankDrive(double rightSpeed, double leftSpeed) {

    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stop() {
    drive.stopMotor();
  }

  public int getRightEncoder() {
    return m_rightEncoder.get();
  }

  public int getLeftEncoder() {
    return m_leftEncoder.get();
  }
}
