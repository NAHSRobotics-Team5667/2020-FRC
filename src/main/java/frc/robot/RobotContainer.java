/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Paths;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
	private static Controller m_controller = new Controller(Constants.ControllerConstants.controllerPort);
	private static DriveTrainSubsystem m_drive;
	private Trajectory trajectory;
	private static ShooterSubsystem m_shooter;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		try {
			trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/circle.wpilib.json"));
			System.out.println("Trajectory loaded successfully");
		} catch (Exception e) {
			try {
				trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(
						"/home/spectre/Desktop/dev/dev-Robotics/FRC_2020/src/main/deploy/output/circle.wpilib.json"));
			} catch (Exception s) {
				trajectory = null;
				System.out.println("Could not load trajectory");
			}
		}

		m_drive = new DriveTrainSubsystem(new PWMTalonSRX(Constants.DriveConstants.fRight),
				new PWMTalonSRX(Constants.DriveConstants.bRight), new PWMTalonSRX(Constants.DriveConstants.fLeft),
				new PWMTalonSRX(Constants.DriveConstants.bLeft),
				new Encoder(Constants.DriveConstants.rEncoderA, Constants.DriveConstants.rEncoderB),
				new Encoder(Constants.DriveConstants.lEncoderA, Constants.DriveConstants.lEncoderB),
				new ADXRS450_Gyro());

		m_shooter = new ShooterSubsystem(new PWMTalonSRX(Constants.ShooterConstants.RightShooter_Port),
				new PWMTalonSRX(Constants.ShooterConstants.LeftShooter_Port),
				new PWMTalonSRX(Constants.ShooterConstants.AngleShooter_Port),
				new Encoder(Constants.ShooterConstants.RightEncoder_Port_A,
						Constants.ShooterConstants.RightEncoder_Port_B),
				new Encoder(Constants.ShooterConstants.LeftEncoder_Port_A,
						Constants.ShooterConstants.LeftEncoder_Port_B),
				new Encoder(Constants.ShooterConstants.AngleEncoder_Port_A,
						Constants.ShooterConstants.AngleEncoder_Port_B));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		if (trajectory != null) {
			RamseteCommand ramseteCommand = new RamseteCommand(
					// The Trajectory
					trajectory,
					// Get the current robot pos
					m_drive::getPose,
					// Ramsete Controller
					new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
					// Feed Forward
					new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
							Constants.DriveConstants.kvVoltSecondsPerMeter,
							Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
					// Drive Kinematics
					Constants.DriveConstants.kDriveKinematics,
					// Get Wheel speeds
					m_drive::getWheelSpeeds,
					// PID Controllers
					new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
					new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
					// RamseteCommand passes volts to the callback
					m_drive::tankDriveVolts, m_drive);

			return ramseteCommand.andThen(m_drive::stop);
		} else {
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

}
