package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
import frc.robot.RobotState.States;
import frc.robot.sensors.Rev2mTOF;

public class IntakeSubsystem extends SubsystemBase {

	private WPI_VictorSPX m_intake;
	private WPI_TalonSRX m_belt;
	private WPI_TalonFX m_colson;
	private Solenoid m_leftSolenoid, m_rightSolenoid;
	public Rev2mTOF tof_sensor = new Rev2mTOF("Intake", Port.kMXP, Unit.kInches, RangeProfile.kHighAccuracy,
			Constants.IntakeConstants.SENSOR_RANGE_INCHES);

	private boolean m_status = false;

	private ShuffleboardTab compTab = Shuffleboard.getTab("Teleop");

	/**
	 * /** Subsystem that handles the intake functionality
	 * 
	 * @param intake    - The intake motor
	 * @param solenoid  - The piston for the left side
	 * @param solenoid2 - The piston for the right side
	 * @param belt      - The motor that controls the belt
	 * @param colson    - The motor that controls the colson wheel on the index
	 */
	public IntakeSubsystem(WPI_VictorSPX intake, Solenoid solenoid, Solenoid solenoid2, WPI_TalonSRX belt,
			WPI_TalonFX colson) {
		m_intake = intake;
		m_belt = belt;
		m_colson = colson;

		m_belt.configFactoryDefault();

		m_intake.setNeutralMode(NeutralMode.Brake);
		m_belt.setNeutralMode(NeutralMode.Brake);

		m_leftSolenoid = solenoid;
		m_rightSolenoid = solenoid2;

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
		m_leftSolenoid.set(Constants.IntakeConstants.SOLENOID_FIRED);
		m_rightSolenoid.set(Constants.IntakeConstants.SOLENOID_FIRED);
		m_intake.set(ControlMode.PercentOutput, Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
		m_colson.set(ControlMode.PercentOutput, .3);
		m_status = Constants.IntakeConstants.SOLENOID_FIRED;
		Constants.m_RobotState.setState(States.INTAKING);
	}

	/**
	 * Extends the intake outside of frame perimeter
	 */
	public void extendIntake(double speed) {
		m_leftSolenoid.set(Constants.IntakeConstants.SOLENOID_FIRED);
		m_rightSolenoid.set(Constants.IntakeConstants.SOLENOID_FIRED);
		m_intake.set(ControlMode.PercentOutput, speed);
		m_colson.set(ControlMode.PercentOutput, .3);
		m_status = Constants.IntakeConstants.SOLENOID_FIRED;
		Constants.m_RobotState.setState(States.INTAKING);
	}

	/**
	 * Retracts the intake inside of frame perimeter
	 */
	public void retractIntake() {
		m_leftSolenoid.set(!Constants.IntakeConstants.SOLENOID_FIRED);
		m_rightSolenoid.set(!Constants.IntakeConstants.SOLENOID_FIRED);
		m_intake.stopMotor();
		m_colson.stopMotor();
		m_status = !Constants.IntakeConstants.SOLENOID_FIRED;
		Constants.m_RobotState.setState(States.DRIVE);
	}

	/**
	 * Start the belt
	 */
	public void startBelt() {
		m_belt.set(ControlMode.PercentOutput, Constants.IntakeConstants.BELT_MOTOR_SPEED);
	}

	/**
	 * Drive the belt at a set speed
	 * 
	 * @param speed - The speed at which to run the belt
	 */
	public void driveBelt(double speed) {
		m_belt.set(ControlMode.PercentOutput, speed);
	}

	/**
	 * Stop the belt
	 */
	public void stopBelt() {
		m_belt.stopMotor();
	}

	/**
	 * Drive the intake at a set speed
	 * 
	 * @param speed - The speed for the intake
	 */
	public void driveIntake(double speed) {
		m_intake.set(-speed);
	}

	/**
	 * Stop the intake motor
	 */
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

	/**
	 * Toggle the intake
	 */
	public void toggle(double speed) {
		if (m_status == Constants.IntakeConstants.SOLENOID_FIRED) {
			retractIntake();
		} else {
			extendIntake(speed);
		}
	}

	/**
	 * Is the intake extended
	 * 
	 * @return is the intake extended
	 */
	public boolean isExtended() {
		return m_status == Constants.IntakeConstants.SOLENOID_FIRED;
	}

	/**
	 * Get the speed the belt is running at
	 * 
	 * @return - The speed the belt index is running at
	 */
	public double getBeltSpeed() {
		return m_belt.getMotorOutputPercent();
	}

	/**
	 * Set the colson wheel to a set speed
	 * 
	 * @param speed - The speed to set the colson wheel
	 */
	public void setColson(double speed) {
		m_colson.set(ControlMode.PercentOutput, speed);
	}

	/**
	 * Stop the colson wheel motor
	 */
	public void stopColson() {
		m_colson.stopMotor();
	}

	/**
	 * Get the colson motor output current
	 * 
	 * @return - The colson motor output current
	 */
	public double getColsonCurrent() {
		return m_colson.getStatorCurrent();
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
