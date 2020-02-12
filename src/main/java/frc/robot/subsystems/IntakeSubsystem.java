package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Rev2mTOF;

public class IntakeSubsystem extends SubsystemBase {

	private WPI_VictorSPX m_intake;
	private Solenoid m_rSolenoid, m_lSolenoid;
	private WPI_VictorSPX m_belt;
	public Rev2mTOF tof_sensor = new Rev2mTOF(Port.kOnboard, Unit.kInches, RangeProfile.kHighAccuracy,
			Constants.IntakeConstants.SENSOR_RANGE_INCHES);

	private ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	/**
	 * Subsystem that handles the intake functionality
	 * 
	 * @param intake    - The intake motor
	 * @param rSolenoid - The right solenoid
	 * @param lSolenoid - The left solenoid
	 * @param belt      - The motor that controls the belt
	 */
	public IntakeSubsystem(WPI_VictorSPX intake, Solenoid rSolenoid, Solenoid lSolenoid, WPI_VictorSPX belt) {
		m_intake = intake;
		m_belt = belt;

		m_intake.setNeutralMode(NeutralMode.Brake);
		m_belt.setNeutralMode(NeutralMode.Brake);

		m_rSolenoid = rSolenoid;
		m_lSolenoid = lSolenoid;

		tof_sensor.enable();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
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

	/**
	 * Toggle the intake
	 */
	public void toggle() {
		if (m_lSolenoid.get() == Constants.IntakeConstants.SOLENOID_FIRED) {
			retractIntake();
		} else {
			extendIntake();
		}
	}

	/**
	 * Output telemtry data
	 */
	public void outputTelemetry() {

		compTab.addBoolean("Intake", new BooleanSupplier() {
			@Override
			public boolean getAsBoolean() {
				return (m_lSolenoid.get() && m_rSolenoid.get()) == Constants.IntakeConstants.SOLENOID_FIRED;
			}
		}).withWidget(BuiltInWidgets.kBooleanBox);
	}

}
