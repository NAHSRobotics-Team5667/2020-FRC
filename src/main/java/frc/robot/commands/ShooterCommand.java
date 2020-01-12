/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterCommand extends CommandBase {
    /**
     * Creates a new ShooterCommand.
     */
    public ShooterCommand() {
        // Use addRequirements() here to declare subsystem dependencies.

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.getController().getAButton() == true) {
            // Fires the shooting wheels when A is pressed
            RobotContainer.getShooterSubsystem().fire(0, 0); /** Replace 0s with proper values **/
        } else if (RobotContainer.getController().getAButton() == false) {
            // Stops firing the shooting wheels when A is released
            RobotContainer.getShooterSubsystem().stopFire();
        }
        if (RobotContainer.getController().getLeftTrigger() >= 2) {
            /**
             * Put trigger code here. Confirm if this needs to be L or R trigger. Also
             * replace "2" for a proper value.
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
