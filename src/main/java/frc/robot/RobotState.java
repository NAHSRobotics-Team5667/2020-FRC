/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
		 */
		IDLE(0), DRIVE(1), AUTO(2), VISION(3), SHOOTING(4), CLIMBING(5), ROTATION(6), POSITION(7);

		private int state;

		/**
		 * A state
		 * 
		 * @param state - The state represented as an integer
		 */
		private States(int state) {
			this.state = state;
		}

		/**
		 * Get the state as an integer
		 * 
		 * @return The state as an integer
		 */
		public int getState() {
			return state;
		}

	}

	public RobotState(States state) {
		if (state != null)
			currentState = state;
		else
			currentState = States.IDLE;
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
