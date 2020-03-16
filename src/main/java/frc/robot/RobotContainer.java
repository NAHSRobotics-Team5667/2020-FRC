/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PATHS;
import frc.robot.Constants.ShooterConstants;
import frc.robot.autos.RunPath;
import frc.robot.autos.ShootAndStay;
import frc.robot.autos.TrenchPathAuto;
import frc.robot.autos.TrenchPathSide;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.actions.AlignCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.ShootAutomatically;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.wheel.PositionCommand;
import frc.robot.commands.wheel.RotationCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WheelSubsystem;
import frc.robot.utils.Controller;
import frc.robot.utils.LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private static Controller m_controller = new Controller(Constants.ControllerConstants.CONTROLLER_PORT);

	private Trajectory[] paths = new Trajectory[] { PATHS.PathWeaver.getTrajectory("FAR_TRENCH"),
			PATHS.PathWeaver.getTrajectory("FAR_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH"),
			PATHS.PathWeaver.getTrajectory("MIDDLE_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("CLOSE_TRENCH"),
			PATHS.PathWeaver.getTrajectory("CLOSE_RENDEVOUS"), PATHS.PathWeaver.getTrajectory("BALL_THIEF"), null,
			PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH_SIDE"), null, PATHS.STRAIGHT_TRAJECTORY_2M,
			PATHS.S_TRAJECTORY };

	public static int ballCount = Constants.IntakeConstants.START_BALL_COUNT;

	private static DriveTrainSubsystem m_drive;
	private static ShooterSubsystem m_shooter;
	private static WheelSubsystem m_wheel;
	private static IntakeSubsystem m_intake;
	private static ClimbSubsystem m_climb;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Instantiation of Subsystems
		m_drive = new DriveTrainSubsystem(new WPI_TalonFX(Constants.DriveConstants.RIGHT_MASTER),
				new WPI_TalonFX(Constants.DriveConstants.LEFT_MASTER),
				new WPI_TalonFX(Constants.DriveConstants.RIGHT_SLAVE),
				new WPI_TalonFX(Constants.DriveConstants.LEFT_SLAVE), new AHRS(SPI.Port.kMXP));

		m_wheel = new WheelSubsystem(new WPI_TalonFX(Constants.WheelConstants.MOTOR));

		m_shooter = new ShooterSubsystem(new WPI_TalonFX(Constants.ShooterConstants.PORT));

		m_intake = new IntakeSubsystem(new WPI_VictorSPX(Constants.IntakeConstants.MOTOR_PORT),
				new Solenoid(Constants.IntakeConstants.SOLENOID_PORT),
				new Solenoid(Constants.IntakeConstants.SOLENOID_2_PORT),
				new WPI_TalonSRX(Constants.IntakeConstants.BELT_PORT),
				new WPI_TalonFX(Constants.IntakeConstants.COLSON_PORT));
		m_climb = new ClimbSubsystem(
				new SpeedControllerGroup(new WPI_VictorSPX(Constants.ClimbConstants.WINCH_MOTOR_1),
						new WPI_VictorSPX(Constants.ClimbConstants.WINCH_MOTOR_2)),
				new Solenoid(Constants.ClimbConstants.HOOK_SOLENOID));

		// Trigger to handle TOF sensor
		m_intake.tof_sensor.trigger.whileActiveOnce(new InstantCommand(() -> {
			if (m_intake.getBeltSpeed() > 0)
				ballCount += 1;
		}));

		// Set default commands
		m_drive.setDefaultCommand(new DriveTrainCommand(m_drive));
		m_intake.setDefaultCommand(new IntakeCommand(m_intake));
		m_shooter.setDefaultCommand(new ShooterCommand(m_shooter));
		m_climb.setDefaultCommand(new ClimbCommand(m_climb));

		// Configure the button bindings
		configureButtonBindings();

		// Output Telemetry
		outputTelemetry();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		Button b = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_B_PORT);
		Button y = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_Y_PORT);

		Button left_stick = new JoystickButton(getController(), Constants.ControllerConstants.S_LEFT);
		Button right_stick = new JoystickButton(getController(), Constants.ControllerConstants.S_RIGHT);

		Button start = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_START_PORT);
		Button menu = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_MENU_PORT);

		y.whenPressed(new AlignCommand(m_drive));

		left_stick.whenPressed(
				() -> LimeLight.getInstance().setPipeline(LimeLight.getInstance().getPipeIndex() == 0 ? 1 : 0));
		right_stick.whenPressed(() -> toggleRPMS());
		b.whenPressed(new ShootAutomatically(m_shooter, m_intake));

		menu.whenPressed(new RotationCommand(m_wheel));
		start.whenPressed(new PositionCommand(m_wheel));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run during autonomous
	 */
	public Command getAutonomousCommand(int selection) {
		if (selection < 6) {
			// Trench Autos
			if (selection % 2 == 0)
				return TrenchPathAuto.getAuto(paths[selection], Constants.PATHS.TRENCH_LINE, m_drive, m_intake,
						m_shooter);
			// Rendevous Autos
			else
				return null;
		} else if (selection == 6) {
			// Ball Thief
			return null;
		} else if (selection == 7) {
			// Code for shoot and cross
			m_drive.resetOdometry(Constants.PATHS.OFF_LINE.getInitialPose());
			return new ShootAndStay(m_shooter, m_drive, m_intake, Constants.PATHS.OFF_LINE,
					ShooterConstants.AUTO_LINE_THRESHOLD, ShooterConstants.AUTO_LINE_DEADBAND);
		} else if (selection > 7 && selection < 10) {
			// Trench Side Autos
			m_drive.resetOdometry(paths[selection].getInitialPose());
			return TrenchPathSide.getAuto(paths[selection], Constants.PATHS.SIDE_FORWARD, m_drive, m_intake, m_shooter);
		} else if (selection > 9 && selection <= paths.length) {
			// Default Path Commands
			m_drive.resetOdometry(paths[selection].getInitialPose());
			return RunPath.getCommand(paths[selection], m_drive, false).andThen(new RunCommand(m_drive::stop));
		} else {
			// Do nothing
			return null;
		}
	}

	/**
	 * Get the controller instance
	 * 
	 * @return The controller instance
	 */
	public static Controller getController() {
		return m_controller;
	}

	/**
	 * Feed the motor safety during auto
	 */
	public void feedMotorSafety() {
		m_drive.feedMotorSafety();
	}

	/**
	 * Set the drive train neutral mode
	 * 
	 * @param mode - The neutral mode to put on drive train
	 */
	public void setNeutralMode(NeutralMode mode) {
		m_drive.setNeutralMode(mode);
	}

	public void toggleRPMS() {
		Constants.ShooterConstants.TRENCH_RPM = (Constants.ShooterConstants.TRENCH_RPM == Constants.ShooterConstants.TRENCH_END_RPM
				? Constants.ShooterConstants.TRENCH_FAR_RPM
				: Constants.ShooterConstants.TRENCH_END_RPM);
	}

	/**
	 * Send telemetry to Shuffleboard
	 */
	public void outputTelemetry() {
		Shuffleboard.getTab("Teleop").addNumber("Count", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return (double) ballCount;
			}
		});

		Shuffleboard.getTab("Teleop").addNumber("Distance", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return LimeLight.getInstance().getDistance(Constants.VisionConstants.H1, Constants.VisionConstants.H2,
						Constants.VisionConstants.A1, LimeLight.getInstance().getYAngle());
			}
		});
		Shuffleboard.getTab("Teleop").addString("TRENCH RPMS", new Supplier<String>() {

			@Override
			public String get() {
				return Constants.ShooterConstants.TRENCH_RPM == Constants.ShooterConstants.TRENCH_END_RPM ? "HIGH"
						: "LOW";
			}
		});
	}

}
