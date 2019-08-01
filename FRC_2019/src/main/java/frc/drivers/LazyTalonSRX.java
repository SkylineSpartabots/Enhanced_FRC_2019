/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * Reduces traffic on CAN bus by only using "set" method when there is a change in
 * mode or value being written to motor controller
 */
public class LazyTalonSRX extends TalonSRX {
    protected double lastSet = Double.NaN;
    protected ControlMode lastControlMode = null;

    public LazyTalonSRX(int deviceID) {
        super(deviceID);
    }

    public double getLastSet() {
        return lastSet;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if(value != lastSet || mode != lastControlMode) {
            lastSet = value;
            lastControlMode = mode;
            super.set(mode, value);
        }
    }

}
