/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveTrainCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
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
	private static Trajectory[] trajectories = { Constants.Autos.PATHS.STRAIGHT_TRAJECTORY,
			Constants.Autos.PATHS.S_TRAJECTORY, Constants.Autos.PATHS.METER, Constants.Autos.PATHS.SEMI_CIRCLE };

	private static Trajectory trajectory = trajectories[3];
	public boolean done = false;

	private static DriveTrainSubsystem m_drive;

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

		m_drive.setDefaultCommand(new DriveTrainCommand(m_drive));

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		Button x = new JoystickButton(getController(), Constants.ControllerConstants.BUTTON_X_PORT);
		x.whenPressed(() -> m_drive.resetOdometry(trajectory.getInitialPose()));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		if (trajectory != null && !done) {
			// Set the initial pos as the current pos
			m_drive.resetOdometry(trajectory.getInitialPose());

			RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drive::getPose,
					new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
					new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
							DriveConstants.kaVoltSecondsSquaredPerMeter),
					DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds, Constants.AutoConstants.L_CONTROLLER,
					Constants.AutoConstants.R_CONTROLLER,
					// RamseteCommand passes volts to the callback
					m_drive::tankDriveVolts, m_drive);
			return ramseteCommand.andThen(new RunCommand(() -> m_drive.drive(0, 0, false)));
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

	public void feedMotorSafety() {
		m_drive.feedMotorSafety();
	}

}
