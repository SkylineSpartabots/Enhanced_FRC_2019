/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.paths;

import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class CurvedProfile {

    private double leftDistance;
    private double rightDistance;
    private boolean isLeft;
    private double timeToExecute;

    private double desiredLeftVelocity, desiredRightVelocity;

    public CurvedProfile(double leftDistance, double rightDistance, boolean isLeft) {
        this(leftDistance, rightDistance, isLeft, Double.NaN);
    }

    public CurvedProfile(double leftDistance, double rightDistance, boolean isLeft, double timeToExecute) {
        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        this.timeToExecute = timeToExecute;

        if(leftDistance > rightDistance) {
            desiredLeftVelocity = Constants.MAX_DRIVE_VELOCITY;
            desiredRightVelocity = (rightDistance/leftDistance) * Constants.MAX_DRIVE_VELOCITY;
        } else {
            desiredRightVelocity = Constants.MAX_DRIVE_VELOCITY;
            desiredLeftVelocity = (leftDistance/rightDistance) * Constants.MAX_DRIVE_VELOCITY;
        }
    }

    public double getRightDistance() {
        return isLeft ? leftDistance : rightDistance;
    }
    
    public double getDesiredRightVelocity() {
        return isLeft ? desiredLeftVelocity : desiredRightVelocity;
    }

    public double getDesiredLeftVelocity() {
        return isLeft ? desiredRightVelocity : desiredLeftVelocity;
    }

    public double getLeftDistance() {
        return isLeft ? rightDistance : leftDistance;
    }

    public double getTimeToExecute() {
        return timeToExecute;
    }




}
