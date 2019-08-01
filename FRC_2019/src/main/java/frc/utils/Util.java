/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

/**
 * Add your docs here.
 */
public class Util {

    public static double deadBand(double val, double deadBand) {
        return (Math.abs(val) > Math.abs(deadBand)) ? val : 0.0;
    }

    public static double boundToScope(double scopeFloor, double scopeCeiling, double value) {
        double stepSize = scopeCeiling - scopeFloor;
        while(value >= scopeCeiling) {value -= stepSize;}
        while(value < scopeFloor) {value += stepSize;}
        return value;
    }

    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

}
