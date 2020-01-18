

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private SpeedController m_lWheel, m_rWheel;

    private SpeedControllerGroup m_intake;

    private Ultrasonic m_ultrasonic;

    private Solenoid m_rSolenoid, m_lSolenoid;

    private int index = 3;

    private boolean previous, current = false;

  public IntakeSubsystem(SpeedController belt, SpeedController intake, Solenoid rSolenoid, Solenoid lSolenoid,
   Ultrasonic ultrasonic) {

    m_belt = belt;

    m_intake = intake;

    m_rSolenoid = rSolenoid;

    m_lSolenoid = lSolenoid;

    m_ultrasonic = ultrasonic;

  }
  public void intake(){
      rSolenoid.set(true);

      lSolenoid.set(true);

      intake.setSpeed(1.0);

  }

  public void retractIntake(){

       rSolenoid.set(false);

       lSolenoid.set(false);

       intake.setspeed(0);
  }

  public boolean hasSeenBall(){
    m_ultrasonic.
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
