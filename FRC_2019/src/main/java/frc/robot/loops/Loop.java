/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.loops;

/**
 * This is an interface for loops. Loops are routines that run periodically in 
 * the robot code
 */
public interface Loop {

    public void onStart(double timestamp);

    public void onLoop(double timestamp);
    
    public void onStop(double timestamp);

}
