/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ClimbCommand extends CommandBase {
  ClimbSubsystem climbSubsystem;

  /**
   * Creates a new ClimbCommand.
   */
  public ClimbCommand(ClimbSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    climbSubsystem = subsystem;
		addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
/*

Haikus with Olu 2:

When the "Y" button's
pressed, the motor moves until 
the target height's reached.


This has been Haikus with Olu.
*/

  if (RobotContainer.getController().getYButtonPressed() == true) {
	
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
