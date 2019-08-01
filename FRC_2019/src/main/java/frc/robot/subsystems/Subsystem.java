/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.loops.ILooper;

/**
 * Add your docs here.
 */
public abstract class Subsystem {
    public void writeToLog() {}

    public void readPeriodicInputs() {}

    public void writePeriodicOutputs() {}

    public abstract void outputTelemetry();

    public abstract void stop();

    public void zeroSensors() {}

    public void registerEnabledLoops(ILooper enabledLooper) {}

    public boolean hasEmergency = false;
}
