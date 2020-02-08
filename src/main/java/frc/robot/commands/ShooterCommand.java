/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState.States;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.PIDFController;

public class ShooterCommand extends CommandBase {

    private PIDFController shooterAngle = new PIDFController("shooterAngle", 0.0, 0.0, 0.0, 0.0);

    private ShooterSubsystem m_shooter;

    /**
     * Creates a new ShooterCommand.
     */
    public ShooterCommand(ShooterSubsystem shooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooter = shooter;
        addRequirements(m_shooter);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Turns on the shooter motor - it should stay on the entire match
        m_shooter.fire(1, 1);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.getController().getLeftTrigger() != 0) {
            shooterAngle.calculate(0.0);

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Turns off the shooter motor at the match's end
        m_shooter.stopFire();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
