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
public class DriveSignal {
    protected double left;
    protected double right;
    protected boolean brakeMode;

    public DriveSignal(double leftMotor, double rightMotor, boolean brakeMode) {
        this.left = leftMotor;
        this.right = rightMotor;
        this.brakeMode = brakeMode;
    }

    public DriveSignal(double leftMotor, double rightMotor) {
        this(leftMotor, rightMotor, false);
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }

    public boolean getBrakeMode() {
        return brakeMode;
    }

    @Override
    public String toString() {
        return "L: " + left + ", R: " + right + (brakeMode ? " , BRAKE" : ", COAST");
    }
    
}
