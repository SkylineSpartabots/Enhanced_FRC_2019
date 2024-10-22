/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

/**
 * Add your docs here.
 */
public interface Action {

    public abstract boolean isFinished();

    public abstract void start();

    public abstract void update();

    public abstract void done();

}
