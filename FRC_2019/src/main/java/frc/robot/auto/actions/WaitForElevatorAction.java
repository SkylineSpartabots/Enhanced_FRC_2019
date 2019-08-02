/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.robot.subsystems.Elevator;

/**
 * Add your docs here.
 */
public class WaitForElevatorAction implements Action {
    private Elevator elevator;
    private double targetHeight;
    private boolean above;

    @Override
    public boolean isFinished() {
        return above ? (elevator.getHeight() > targetHeight) : (elevator.getHeight() < targetHeight);
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
