/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

///** Make sure to change things based on updates to ShooterSubsystem and elsewhere

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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
        RobotContainer.getShooterSubsystem().fire(0, 0); /** Replace 0s with proper values **/

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.getController().getAButton() == true) {
            // *Add method for activating the conveyor belt here
        }
        if (RobotContainer.getController().getLeftTrigger() != 0) {
            shooterAngle.calculate(0.0);
            /**
             * This method should use vision to automatically adjust the shooting angle
             **/
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
