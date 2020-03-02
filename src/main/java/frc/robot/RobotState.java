/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.LedConstants.Colors;

/**
 * Robot State Machine
 */
public class RobotState {

	private static States currentState;

	public static enum States {
		/**
		 * IDLE - The robot is currently not doing anything
		 * 
		 * DRIVE - The robot is not doing anything else but driving
		 * 
		 * AUTO - The robot is in autonomous mode
		 * 
		 * VISION - The robot is currently using vision tracking
		 * 
		 * SHOOTING - The robot is currently trying to shoot a POWER CELL
		 * 
		 * CLIMBING - The robot is currently attempting to/is climbing
		 * 
		 * ROTATION - The robot is currently attempting Rotation Control
		 * 
		 * POSITION - The robot is currently attempting Position Control
		 * 
		 * INTAKE - The robot is currently intaking
		 */
		IDLE(0, Constants.LedConstants.Colors.RED), DRIVE(1, Constants.LedConstants.Colors.BLUE),
		ALIGNING(2, Constants.LedConstants.Colors.RED), ALIGNED(3, Constants.LedConstants.Colors.YELLOW),
		REVED(4, Constants.LedConstants.Colors.GREEN), INTAKING(5, Constants.LedConstants.Colors.PINK);

		private int state;
		private int[] color;

		/**
		 * A state
		 * 
		 * @param state - The state represented as an integer
		 * @param color - The Color associated with the state
		 */
		private States(int state, Colors color) {
			this.state = state;
			this.color = color.getColor();
		}

		/**
		 * Get the state as an integer
		 * 
		 * @return The state as an integer
		 */
		public int getState() {
			return state;
		}

		public int[] getColor() {
			return color;
		}

		public String getString() {
			switch (state) {
			case 0:
				return "IDLE";
			case 1:
				return "DRIVE";
			case 2:
				return "AlIGNING";
			case 3:
				return "ALIGNED";
			case 4:
				return "REVED";
			case 5:
				return "INTAKING";
			case 6:
				return "POSITION";
			case 7:
				return "INTAKING";
			default:
				return "IDLE";
			}
		}
	}

	public RobotState(States state) {
		if (state != null)
			currentState = state;
		else
			currentState = States.IDLE;

		Shuffleboard.getTab("Teleop").addString("Robot State", new Supplier<String>() {
			@Override
			public String get() {
				return currentState.getString();
			}
		});
	}

	/**
	 * Get the Robot's current state
	 * 
	 * @return The Robot's current state
	 */
	public States getCurrentState() {
		return currentState;
	}

	/**
	 * Set the current state of the robot
	 * 
	 * @param state - The state the robot should be in
	 */
	public void setState(States state) {
		currentState = state;
	}

}