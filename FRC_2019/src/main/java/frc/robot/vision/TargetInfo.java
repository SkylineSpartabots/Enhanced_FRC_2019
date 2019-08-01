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
    private double xOffset;
    private double targetArea;
    private boolean isTargetVisible;

    public void setXOffset(double xOffset) {
        this.xOffset = xOffset;
    }

    public void setTargetArea(double targetArea) {
        this.targetArea = targetArea;
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


}
