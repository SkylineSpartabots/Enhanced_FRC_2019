/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class WaitToFinishPathAction implements Action {
    private Drivetrain drive;
    private double timeout;
    private double startTime;

    public WaitToFinishPathAction(double timeout) {
        drive = Drivetrain.getInstance();
        this.timeout = timeout;
    }

    @Override
    public boolean isFinished() {
        return //drive.isDoneWithTrajectory() || 
        (Timer.getFPGATimestamp() - startTime) > timeout;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }
}
