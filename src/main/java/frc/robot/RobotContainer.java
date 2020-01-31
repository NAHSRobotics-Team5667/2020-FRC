/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveTrainCommand;
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
	private static Controller m_controller = new Controller(Constants.ControllerConstants.CONTROLLER_PORT);
	private static DriveTrainSubsystem m_drive;
	private static ShooterSubsystem m_shooter;

	private Trajectory trajectory;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		try {
			trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/circle.wpilib.json"));
		} catch (Exception e) {
			trajectory = null;
			System.out.println("Could not load trajectory");
		}

		// m_drive = new DriveTrainSubsystem(new
		// WPI_TalonSRX(Constants.DriveConstants.RIGHT_MASTER),
		// new WPI_TalonSRX(Constants.DriveConstants.LEFT_MASTER),
		// new WPI_TalonSRX(Constants.DriveConstants.RIGHT_SLAVE),
		// new WPI_TalonSRX(Constants.DriveConstants.LEFT_SLAVE), new
		// AHRS(SPI.Port.kMXP));

		// m_drive.setDefaultCommand(new DriveTrainCommand(m_drive));
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
			// Set the initial pos as the current pos
			m_drive.resetOdometry(trajectory.getInitialPose());

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
					new PIDController(Constants.DriveConstants.kPDrive, 0, Constants.DriveConstants.kDDrive),
					new PIDController(Constants.DriveConstants.kPDrive, 0, Constants.DriveConstants.kDDrive),
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
