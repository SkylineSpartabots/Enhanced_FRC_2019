/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.drivers;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class LazySolenoid extends Solenoid {

    protected Boolean state = null;

    public LazySolenoid(int deviceID) {
        super(deviceID);
    }

    public boolean getLastState() {
        return state;
    }

    @Override
    public void set(boolean on) {
        if(state != on) {
            state = on;
            super.set(on);
        }
    }

}
