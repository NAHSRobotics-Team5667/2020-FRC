/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ResetIndexCommand extends CommandBase {
	private IntakeSubsystem m_intake;
	private ShooterSubsystem m_shooter;
	private Timer timer;

	/**
	 * Creates a new ResetIndexCommand.
	 */
	public ResetIndexCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_intake = intake;
		m_shooter = shooter;
		addRequirements(m_intake, m_shooter);
		timer = new Timer();
		timer.reset();
		timer.start();

		System.out.println("STARTED RESET INDEX");
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_intake.stopBelt();
		m_shooter.stopFire();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_intake.driveBelt(-.6);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_intake.stopBelt();
		m_intake.stopIntakeMotor();
		m_shooter.stopFire();
		System.out.println("RESET INDEX ENDED");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.hasPeriodPassed(5) && m_intake.tof_sensor.isDetecting();
	}
}
