/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * Add your docs here.
 */
public class CurrentSpikeCounter {
    private double threshold;
    private double deadband;
    private double off_val;

    private boolean isSpiking = false;
    private boolean ramped = false;
    private boolean isRamping = false;

    public CurrentSpikeCounter(double t, double d) {
        threshold = t;
        deadband = d;
        off_val = threshold - deadband;

    }

    public boolean update(double current) {
        if (!ramped && current > threshold) {
            isRamping = true;
            return false;
        } else if (isRamping && current < off_val) {
            ramped = true;
            isRamping = false;
            return false;
        } else if (ramped && current > threshold && !isSpiking) {
            isSpiking = true;
            return true;
        } else if (ramped && current < off_val && isSpiking) {
            isSpiking = false;
            return false;
        } else {
            return false;
        }

    }

    public boolean hasRamped() {
        return ramped;
    }

    public boolean isRamping() {
        return isRamping;
    }

    public boolean isSpiking() {
        return isSpiking;
    }
}
