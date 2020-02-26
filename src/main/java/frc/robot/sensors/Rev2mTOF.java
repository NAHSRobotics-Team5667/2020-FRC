/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Rev2mTOF {

	private Rev2mDistanceSensor sensor;
	private double threshold;

	private boolean lastStatus = false;
	private boolean currentStatus = false;
	private boolean notChecked = true;

	public Trigger trigger = new Trigger(new BooleanSupplier() {

		@Override
		public boolean getAsBoolean() {
			update();
			return isDetecting();
		}
	});

	private String name;

	private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");

	/**
	 * Create a Rev 2 Meter Distance Sensor Trigger
	 * 
	 * @param port         - The port the sensor is connected to
	 * @param units        - The units to measure the sensor in
	 * @param rangeProfile - Range Mode
	 */
	public Rev2mTOF(String name, Port port, Unit units, RangeProfile rangeProfile, double threshold) {
		sensor = new Rev2mDistanceSensor(port, units, rangeProfile);
		this.threshold = threshold;
		this.name = name;
		this.sensor.setAutomaticMode(true);
		this.sensor.setEnabled(true);
	}

	/**
	 * Create a Rev 2 Meter Distance Sensor Trigger
	 * 
	 * @param port - The port the sensor is connected to
	 */
	public Rev2mTOF(Port port, double threshold) {
		sensor = new Rev2mDistanceSensor(port);
		this.threshold = threshold;
	}

	/**
	 * Get the Rev2mDistanceSensor instance
	 */
	public Rev2mDistanceSensor getSensor() {
		return sensor;
	}

	/**
	 * Has the sensor detected something that is lower than the provided threshold
	 * 
	 * @return Is an object currently detected
	 */
	public boolean isDetecting() {
		return sensor.getRange() <= threshold;
	}

	/**
	 * Has an object passed with the threshold provided
	 * 
	 * @return Whether or not an object has passed through from the provided
	 *         threshold
	 */
	public boolean hasPassed() {
		if (notChecked) {
			notChecked = false;
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Update the sensor values
	 */
	public void update() {
		lastStatus = currentStatus;
		currentStatus = isDetecting();
		if (lastStatus && !currentStatus)
			notChecked = true;
	}

	/**
	 * Enable the sensor
	 */
	public void enable() {
		sensor.setEnabled(true);
	}

	/**
	 * Disable the sensor
	 */
	public void disable() {
		sensor.setEnabled(false);
	}

	public void outputTelemetry() {
		tab.addNumber(name + "_REV", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return sensor.getRange();
			}
		});
	}
}
