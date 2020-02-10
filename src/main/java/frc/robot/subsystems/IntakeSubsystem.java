package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

	private WPI_VictorSPX m_intake;
	private Solenoid m_rSolenoid, m_lSolenoid;
	private WPI_VictorSPX m_belt;
	private Rev2mDistanceSensor m_intakeSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches,
			RangeProfile.kHighAccuracy);

	private ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	private boolean previousSeenBallEnter = false;

	/**
	 * Creates an intake subsystem instance
	 * 
	 * @param intake        - motor controller for the intake wheels
	 * @param rSolenoid     - right piston for extending/retracting intake
	 * @param lSolenoid     - left piston for extending/retracting intake
	 * @param belt          - motor controller for the belt
	 * @param intakeSensor  - Rev2mDistanceSensor that detects if a ball has entered
	 *                      the robot
	 * @param shooterSensor - Rev2mDistanceSensor that detects if a ball has exited
	 *                      the robot
	 */
	public IntakeSubsystem(WPI_VictorSPX intake, Solenoid rSolenoid, Solenoid lSolenoid, WPI_VictorSPX belt) {
		m_intake = intake;
		m_belt = belt;

		m_intake.setNeutralMode(NeutralMode.Brake);
		m_belt.setNeutralMode(NeutralMode.Brake);

		m_rSolenoid = rSolenoid;
		m_lSolenoid = lSolenoid;
	}

	/**
	 * Extends the intake outside of frame perimeter
	 */
	public void extendIntake() {
		m_rSolenoid.set(Constants.IntakeConstants.SOLENOID_FIRED);
		m_lSolenoid.set(Constants.IntakeConstants.SOLENOID_FIRED);
		m_intake.set(Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
	}

	/**
	 * Retracts the intake inside of frame perimeter
	 */
	public void retractIntake() {
		m_rSolenoid.set(!Constants.IntakeConstants.SOLENOID_FIRED);
		m_lSolenoid.set(!Constants.IntakeConstants.SOLENOID_FIRED);
		m_intake.stopMotor();
	}

	/**
	 * Determine if the Rev2mDistanceSensor saw a power cell enter the robot or not
	 * 
	 * @return true if ball is present, therefore seen entering robot
	 */
	public boolean hasSeenBallEnter() {
		boolean currentSeenBallEnter = false;
		if (m_intakeSensor.getRange(Unit.kInches) <= Constants.IntakeConstants.SENSOR_RANGE_INCHES
				&& previousSeenBallEnter == false) {
			currentSeenBallEnter = true;
			previousSeenBallEnter = currentSeenBallEnter;
		} else if (m_intakeSensor.getRange(Unit.kInches) > Constants.IntakeConstants.SENSOR_RANGE_INCHES
				&& previousSeenBallEnter == true) {
			currentSeenBallEnter = false;
			previousSeenBallEnter = currentSeenBallEnter;
		}
		return currentSeenBallEnter;
	}

	/**
	 * Start the belt
	 */
	public void startBelt() {
		m_belt.set(Constants.IntakeConstants.BELT_MOTOR_SPEED);
	}

	/**
	 * Stop the belt
	 */
	public void stopBelt() {
		m_belt.stopMotor();
	}

	public void toggle() {
		if (m_lSolenoid.get() == Constants.IntakeConstants.SOLENOID_FIRED) {
			retractIntake();
		} else {
			extendIntake();
		}
	}

	public void outputIntakeStats() {
		compTab.add("Right Solenoid Fired", m_rSolenoid.get());
		compTab.add("Left Solenoid Fired", m_lSolenoid.get());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}
