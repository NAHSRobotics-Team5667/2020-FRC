
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private SpeedController m_intake;
  private Solenoid m_rSolenoid, m_lSolenoid;
  private SpeedController m_belt;
  private Ultrasonic m_ultrasonic;

  private boolean seenBall = false;

  /**
   * Creates an intake subsystem instance
   * 
   * @param intake     - motor controller for the intake wheels
   * @param rSolenoid  - right piston for extending/retracting intake
   * @param lSolenoid  - left piston for extending/retracting intake
   * @param belt       - motor controller for the belt
   * @param ultrasonic - ultrasonic sensor that detects if the belt needs to be
   *                   activated
   */
  public IntakeSubsystem(SpeedController intake, Solenoid rSolenoid, Solenoid lSolenoid, SpeedController belt,
      Ultrasonic ultrasonic) {
    m_intake = intake;
    m_rSolenoid = rSolenoid;
    m_lSolenoid = lSolenoid;
    m_belt = belt;
    m_ultrasonic = ultrasonic;
  }

  /**
   * Extends the intake outside of frame perimeter
   */
  public void extendIntake() {
    m_rSolenoid.set(true);
    m_lSolenoid.set(true);
    m_intake.set(1.0);
  }

  /**
   * Retracts the intake inside of frame perimeter
   */
  public void retractIntake() {
    m_rSolenoid.set(false);
    m_lSolenoid.set(false);
    m_intake.set(0);
  }

  /**
   * Determine if the ultrasonic sensor sees a power cell or not
   * 
   * @return true if ball is seen
   */
  public boolean hasSeenBall() {
    if (m_ultrasonic.getRangeInches() <= 6 && seenBall == false) {
      seenBall = true;
    } else if (m_ultrasonic.getRangeInches() > 6 && seenBall == true) {
      seenBall = false;
    }
    return seenBall;
  }

  /**
   * Start the belt
   */
  public void startBelt() {
    m_belt.set(1.0);
  }

  /**
   * Stop the belt
   */
  public void stopBelt() {
    m_belt.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}