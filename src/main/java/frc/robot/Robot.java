/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState.States;
import frc.robot.utils.LED;
import frc.robot.utils.LimeLight;
import frc.robot.utils.LimeLight.LightMode.SnapMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand = null;
	private RobotContainer m_robotContainer;

	private ShuffleboardTab compTab = Shuffleboard.getTab("Teleop");

	private SendableChooser<Integer> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		Shuffleboard.selectTab("Teleop");
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(240, 180);
		camera.setFPS(18);
		Shuffleboard.getTab("Teleop").add("Intake Cam", camera);

		m_robotContainer = new RobotContainer();
		m_robotContainer.setNeutralMode(NeutralMode.Coast);
		m_chooser.setDefaultOption("Far Side Trench", 0);
		m_chooser.addOption("Far Rendevous", 1);
		m_chooser.addOption("Middle Trench", 2);
		m_chooser.addOption("Middle Rendevous", 3);
		m_chooser.addOption("Close Trench", 4);
		m_chooser.addOption("Close Rendevous", 5);
		m_chooser.addOption("Ball Theif", 6);
		m_chooser.addOption("Shoot & Stay", 7);
		m_chooser.addOption("Straight 2M", 8);
		m_chooser.addOption("S Path", 9);
		m_chooser.addOption("Middle Side Trench", 10);
		m_chooser.addOption("Far Side Trench", 11);

		m_chooser.addOption("Null", 99);

		compTab.add("Auto Chooser", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);

		LimeLight.getInstance().turnLightOn();

	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		LED.getLEDInstance().setColor();
		if (LimeLight.getInstance().hasValidTarget()) {
			LimeLight.getInstance().takeSnapshots(SnapMode.ENABLED);
		} else {
			LimeLight.getInstance().takeSnapshots(SnapMode.DISABLED);
		}

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
		m_robotContainer.setNeutralMode(NeutralMode.Brake);
		LimeLight.getInstance().setPipeline(0);
		Constants.m_RobotState.setState(States.IDLE);
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		Shuffleboard.selectTab("Teleop");
		m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected());

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		m_autonomousCommand.schedule();
		m_robotContainer.feedMotorSafety();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		Shuffleboard.selectTab("Teleop");
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		m_robotContainer.setNeutralMode(NeutralMode.Coast);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

}
