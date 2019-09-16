/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.actions.Action;
/**
 * Add your docs here.
 */
public abstract class AutoModeBase {
    protected double updateRate = 1.0/50.0;
    protected boolean active = false;

    protected double startTime = 0.0;
    protected double currentTime() {
        return Timer.getFPGATimestamp() - startTime;
    }

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        active = true;
        try {
            routine();
        } catch(AutoModeEndedException e) {
            System.out.println("Auto mode ended early");
            return;
        }

        done();
        System.out.println("Auto mode done");
    }

    public void done() {}

    public void stop() {
        active = false;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if(!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        action.start();
        
        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (updateRate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }
 }
