/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LoadCommand;
import frc.robot.commands.ShootAutonomously;
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

	public boolean done = false;
	public static int ballCount = 3;

	private static DriveTrainSubsystem m_drive;
	private static ShooterSubsystem m_shooter;
	private static WheelSubsystem m_wheel;
	private static IntakeSubsystem m_intake;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		m_drive = new DriveTrainSubsystem(new WPI_TalonFX(Constants.DriveConstants.rightMaster),
				new WPI_TalonFX(Constants.DriveConstants.leftMaster),
				new WPI_TalonFX(Constants.DriveConstants.rightSlave),
				new WPI_TalonFX(Constants.DriveConstants.leftSlave), new AHRS(SPI.Port.kMXP));

		// m_wheel = new WheelSubsystem(new WPI_TalonFX(Constants.WheelConstants.MOTOR),
		// 		new ColorSensorV3(Constants.WheelConstants.COLOR_SENSOR_PORT));

		// m_shooter = new ShooterSubsystem(new WPI_TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_PORT),
		// 		new WPI_TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_PORT));

		m_intake = new IntakeSubsystem(new WPI_VictorSPX(Constants.IntakeConstants.MOTOR_PORT),
				new Solenoid(Constants.IntakeConstants.R_SOLENOID), new Solenoid(Constants.IntakeConstants.L_SOLENOID),
				new WPI_VictorSPX(Constants.IntakeConstants.BELT_PORT));

		m_drive.setDefaultCommand(new DriveTrainCommand(m_drive));

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		final Button x = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_X_PORT);
		final Button b = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_B_PORT);
		final Button a = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_A_PORT);

		x.whenPressed(() -> m_drive.resetOdometry(new Pose2d()));
		a.whenPressed(() -> m_intake.toggle());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand(Trajectory path) {
		// Set the initial pos as the current pos
		m_drive.resetOdometry(path.getInitialPose());

		RamseteCommand ramseteCommand = new RamseteCommand(path, m_drive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
						DriveConstants.kaVoltSecondsSquaredPerMeter),
				DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds, Constants.AutoConstants.L_CONTROLLER,
				Constants.AutoConstants.R_CONTROLLER,
				// RamseteCommand passes volts to the callback
				m_drive::tankDriveVolts, m_drive);

		ShootAutonomously step1 = new ShootAutonomously(m_shooter, m_intake, Constants.ShooterConstants.AUTO_LINE_RPM);
			
		ParallelCommandGroup step2 = new ParallelCommandGroup(new Command[]{ramseteCommand, new InstantCommand(() -> 
				{m_shooter.setSetpoint(Constants.ShooterConstants.TRENCH_RPM); m_shooter.enable();}), new LoadCommand(m_intake, 3)
		});

		ParallelCommandGroup step3 = new ParallelCommandGroup(new Command[]{new ShootAutonomously(m_shooter, m_intake, Constants.ShooterConstants.TRENCH_RPM) });

			return new SequentialCommandGroup(new Command[]{step1, step2, step3});
			// return ramseteCommand.andThen(new RunCommand(() -> m_drive.drive(0, 0, false)));
	}

	/**
	 * Get the controller instance
	 * 
	 * @return The controller instance
	 */
	public static Controller getController() {
		return m_controller;
	}

	public void feedMotorSafety() {
		m_drive.feedMotorSafety();
	}

}
