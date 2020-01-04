/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.states;


/**
 * Wrapper class for describing the state of the rgb LEDs
 */

public class LEDState {
    public double red, green, blue;

    public static final LEDState off = new LEDState(0.0, 0.0, 0.0);
    public static final LEDState exampleState = new LEDState(0.0, 0.3, 1.0);

    public LEDState(double r, double g, double b) {
        red = r;
        green = g;
        blue = b;
    }

    public void copy(LEDState other) {
        this.red = other.red;
        this.green = other.green;
        this.blue = other.blue;
    }


}
