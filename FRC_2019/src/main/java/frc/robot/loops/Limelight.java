/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.loops;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;

/**
 * Add your docs here.
 */
public class Limelight implements Loop {

    private static Limelight limelight;
    public static Limelight getInstance() {
        if(limelight == null) {
            limelight = new Limelight();
        }
        return limelight;
    }

    private NetworkTable limelightTable;
    private NetworkTableEntry ledMode, pipeline, camMode, stream;
    private List<NetworkTableEntry> target;

    private boolean updateVision = true;
    public void enableUpdates(boolean enable) {
        updateVision = enable;
    }

    private Limelight() {

    }

    @Override
    public void onStart(double timestamp) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        ledMode = limelightTable.getEntry("ledMode");
        pipeline = limelightTable.getEntry("pipeline");
        camMode = limelightTable.getEntry("camMode");
        stream = limelightTable.getEntry("stream");
        
        target = Arrays.asList(limelightTable.getEntry("tx"), 
            limelightTable.getEntry("ta"), limelightTable.getEntry("tv"));
    }

    @Override
    public void onLoop(double timestamp) {

        if(updateVision) {
            if(isTargetVisible()) {
                RobotState.visionTarget.setTargetVisible(true);
                RobotState.visionTarget.setTargetArea(target.get(1).getDouble(0));
                RobotState.visionTarget.setXOffset(target.get(0).getDouble(0));
            } else {
                RobotState.visionTarget.setTargetVisible(false);
                RobotState.visionTarget.setTargetArea(0);
                RobotState.visionTarget.setXOffset(0);
            }
        }

    }

    @Override
    public void onStop(double timestamp) {

    }


    private boolean isTargetVisible() {
        boolean isTargetVisible = (target.get(2).getDouble(0) == 1.0) ? true : false;
        return isTargetVisible;
    }
}
