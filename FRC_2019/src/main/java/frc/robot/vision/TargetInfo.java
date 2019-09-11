/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

/**
 * Add your docs here.
 */
public class TargetInfo {

    private double desiredTargetArea = 9;
    public static final double DESIRED_X_OFFSET = 0;
    public static final double TARGET_AREA_THRESHOLD = 1.5;
    public static final double X_OFFSET_THRESHOLD = 8;

    private double xOffset;
    private double targetArea;
    private boolean isTargetVisible;

    public void setXOffset(double xOffset) {
        this.xOffset = xOffset;
    }

    public void setTargetArea(double targetArea) {
        this.targetArea = targetArea;
    }

    public void setDesiredTargetArea(double desiredTargetArea) {
        this.desiredTargetArea = desiredTargetArea;
    }

    public void setTargetVisible(boolean isTargetVisible) {
        this.isTargetVisible = isTargetVisible;
    }

    public double getXOffset() {
        return xOffset;
    }

    public double getTargetArea() {
        return targetArea;
    }

    public boolean isTargetVisible() {
        return isTargetVisible;
    }

    public boolean hasReachedTarget() {
        return Math.abs(getTargetArea() - desiredTargetArea) <= TARGET_AREA_THRESHOLD 
            && Math.abs(getXOffset() - DESIRED_X_OFFSET) <= X_OFFSET_THRESHOLD;
    }


}
