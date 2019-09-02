/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class WaitForHeadingAction implements Action {

    private Drivetrain driveTrain;
    private double lowThreshold;
    private double highThreshold;

    public WaitForHeadingAction(double lowThreshold, double highThreshold) {
        driveTrain = Drivetrain.getInstance();
        this.lowThreshold = lowThreshold;
        this.highThreshold = highThreshold;
    }

    @Override
    public boolean isFinished() {
        double heading = 0;//driveTrain.getHeading().getDegrees();
        return heading >= lowThreshold && heading <= highThreshold;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}
