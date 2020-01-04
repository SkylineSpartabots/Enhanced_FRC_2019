/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.drivers;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

/**
 * Lazy CANifier
 */
public class LazyCANifier extends CANifier {
    private double currentRpow;
    private double currentGpow;
    private double currentBpow;
    public LazyCANifier(int device_id) {
        super(device_id);
    }

    @Override
    public void setLEDOutput(double percentOutput, LEDChannel ledChannel) {
        if(ledChannel.equals(LEDChannel.LEDChannelA) && percentOutput != currentRpow) {
            super.setLEDOutput(percentOutput, ledChannel);
            currentRpow = percentOutput;
        } else if (ledChannel.equals(LEDChannel.LEDChannelB) && percentOutput != currentGpow) {
            super.setLEDOutput(percentOutput, ledChannel);
            currentGpow = percentOutput;
        } else if(ledChannel.equals(LEDChannel.LEDChannelC) && percentOutput != currentBpow) {
            super.setLEDOutput(percentOutput, ledChannel);
            currentBpow = percentOutput;
        }
    }
}