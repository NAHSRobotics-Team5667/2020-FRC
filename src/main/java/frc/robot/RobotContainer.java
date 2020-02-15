/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PATHS;
import frc.robot.autos.RunPath;
import frc.robot.autos.TrenchPathAuto;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.actions.AlignCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WheelSubsystem;
import frc.robot.utils.Controller;

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
			PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH"), PATHS.PathWeaver.getTrajectory("CLOSE_TRENCH"), null,
			PATHS.STRAIGHT_TRAJECTORY_2M, PATHS.S_TRAJECTORY, PATHS.PathWeaver.getTrajectory("MIDDLE_TRENCH") };

	public boolean done = false;
	public static int ballCount = Constants.IntakeConstants.START_BALL_COUNT;

	private static DriveTrainSubsystem m_drive;
	private static ShooterSubsystem m_shooter;
	private static WheelSubsystem m_wheel;
	private static IntakeSubsystem m_intake;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_drive = new DriveTrainSubsystem(new WPI_TalonFX(Constants.DriveConstants.RIGHT_MASTER),
				new WPI_TalonFX(Constants.DriveConstants.LEFT_MASTER),
				new WPI_TalonFX(Constants.DriveConstants.RIGHT_SLAVE),
				new WPI_TalonFX(Constants.DriveConstants.LEFT_SLAVE), new AHRS(SPI.Port.kMXP));

		// m_wheel = new WheelSubsystem(new WPI_TalonFX(Constants.WheelConstants.MOTOR),
		// new ColorSensorV3(Constants.WheelConstants.COLOR_SENSOR_PORT));

		m_shooter = new ShooterSubsystem(new WPI_TalonFX(Constants.ShooterConstants.PORT));

		m_intake = new IntakeSubsystem(new WPI_VictorSPX(Constants.IntakeConstants.MOTOR_PORT),
				new Solenoid(Constants.IntakeConstants.SOLENOID_PORT));

		m_drive.setDefaultCommand(new DriveTrainCommand(m_drive));
		m_shooter.setDefaultCommand(new ShooterCommand(m_shooter));

		// Configure the button bindings
		configureButtonBindings();

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		Button a = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_A_PORT);
		Button b = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_B_PORT);
		Button x = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_X_PORT);
		Button y = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_Y_PORT);

		a.whenPressed(() -> m_intake.toggle());
		y.whenPressed(new AlignCommand(m_drive));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run during autonomous
	 */
	public Command getAutonomousCommand(int selection) {

		if (selection <= 3)
			return TrenchPathAuto.getAuto(paths[selection], m_drive, m_intake, m_shooter);
		else if (selection > 3 && selection < paths.length)
			return RunPath.getCommand(paths[selection], m_drive, true).andThen(() -> {
				m_drive.setNeutralMode(NeutralMode.Brake);
				m_drive.drive(0, 0, false);
			});
		else
			return null;
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

}
