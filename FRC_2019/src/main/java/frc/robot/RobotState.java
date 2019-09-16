/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.SmartDashboardInteractions.DriveStation;
import frc.robot.vision.TargetInfo;


/**
 * Add your docs here.
 */
public class RobotState {
    private static RobotState instance = null;
    
    public static RobotState getInstance() {
        if(instance == null) 
            instance = new RobotState();
        return instance;
    }
    
    public static TargetInfo visionTarget = new TargetInfo();

    private DriveStation side;

    private RobotState() {
        
    }

    public void setAutoSide(DriveStation side) {
        this.side = side;
    }

    public DriveStation getSide() {
        return side;
    }
    public void outputToSmartDashboard() {
        //Pose2d odometry = getLatestFieldToVehicle().getValue();
        //SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
        //SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
        //SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
        //SmartDashboard.putNumber("Robot Linear Velocity", measuredVelocity.dx);
        SmartDashboard.putBoolean("Is Target Visible", visionTarget.isTargetVisible());
        SmartDashboard.putNumber("Target Area", visionTarget.getTargetArea());
        //SmartDashboard.putNumber("Target Angular Displacement", visionTarget.getXOffset());
    }



}
