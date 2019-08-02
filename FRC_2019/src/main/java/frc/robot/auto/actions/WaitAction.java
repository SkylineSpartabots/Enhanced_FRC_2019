/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class WaitAction implements Action {

    private double waitTime;
    private double startTime;

    public WaitAction(double waitTime) {
        this.waitTime = waitTime;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= waitTime;
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
