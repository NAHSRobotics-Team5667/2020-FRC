package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
	private Solenoid m_solenoid, m_solenoid2;
	private WPI_TalonSRX m_belt;
	public Rev2mTOF tof_sensor = new Rev2mTOF("Intake", Port.kMXP, Unit.kInches, RangeProfile.kHighAccuracy,
			Constants.IntakeConstants.SENSOR_RANGE_INCHES);

	private boolean m_status = false;

	private ShuffleboardTab compTab = Shuffleboard.getTab("Teleop");

	/**
	 * Subsystem that handles the intake functionality
	 * 
	 * @param intake    - The intake motor
	 * @param rSolenoid - The right solenoid
	 * @param lSolenoid - The left solenoid
	 * @param belt      - The motor that controls the belt
	 */
	public IntakeSubsystem(WPI_VictorSPX intake, Solenoid solenoid, Solenoid solenoid2, WPI_TalonSRX belt) {
		m_intake = intake;
		m_belt = belt;

		VictorSPXConfiguration config = new VictorSPXConfiguration();

		m_intake.configAllSettings(config);

		m_intake.setNeutralMode(NeutralMode.Brake);
		m_belt.setNeutralMode(NeutralMode.Brake);

		SupplyCurrentLimitConfiguration current_limit = new SupplyCurrentLimitConfiguration();
		current_limit.enable = true;
		current_limit.currentLimit = 25;

		m_belt.configSupplyCurrentLimit(current_limit);

		m_solenoid = solenoid;
		m_solenoid2 = solenoid2;

		outputTelemetry();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/**
	 * Extends the intake outside of frame perimeter
	 */
	public void extendIntake() {
		m_solenoid.set(Constants.IntakeConstants.SOLENOID_FIRED);
		m_solenoid2.set(Constants.IntakeConstants.SOLENOID_FIRED);
		m_intake.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
		m_status = Constants.IntakeConstants.SOLENOID_FIRED;
	}

	/**
	 * Retracts the intake inside of frame perimeter
	 */
	public void retractIntake() {
		m_solenoid.set(!Constants.IntakeConstants.SOLENOID_FIRED);
		m_solenoid2.set(!Constants.IntakeConstants.SOLENOID_FIRED);
		m_intake.stopMotor();
		m_status = !Constants.IntakeConstants.SOLENOID_FIRED;
	}

	/**
	 * Start the belt
	 */
	public void startBelt() {
		m_belt.set(Constants.IntakeConstants.BELT_MOTOR_SPEED);
	}

	/**
	 * Drive the belt at a set speed
	 * 
	 * @param speed - The speed at which to run the belt
	 */
	public void driveBelt(double speed) {
		m_belt.set(speed);
	}

	/**
	 * Stop the belt
	 */
	public void stopBelt() {
		m_belt.stopMotor();
	}

	public void driveIntake(double speed) {
		m_intake.set(-speed);
	}

	public void stopIntakeMotor() {
		m_intake.stopMotor();
	}

	/**
	 * Toggle the intake
	 */
	public void toggle() {
		if (m_status == Constants.IntakeConstants.SOLENOID_FIRED) {
			retractIntake();
		} else {
			extendIntake();
		}
	}

	public boolean isExtended() {
		return m_status == Constants.IntakeConstants.SOLENOID_FIRED;
	}

	public double getBeltSpeed() {
		return m_belt.get();
	}

	/**
	 * Output telemtry data
	 */
	public void outputTelemetry() {

		compTab.addBoolean("Intake", new BooleanSupplier() {
			@Override
			public boolean getAsBoolean() {
				return m_status == Constants.IntakeConstants.SOLENOID_FIRED;
			}
		}).withWidget(BuiltInWidgets.kBooleanBox);
	}

}
