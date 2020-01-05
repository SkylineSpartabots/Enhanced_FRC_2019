/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.Subsystem;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.states.LEDState;
import frc.robot.states.TimedLEDState.SteadyLEDState;
import frc.robot.states.TimedLEDState.FlashingLEDState;
import frc.drivers.LazyCANifier;

/**
 * CANifier Subsystem:
 * EXTREMELY Simple
 */
public class LED extends Subsystem {

  public static LED instance = null;
  public static LED getInstance() {
    if(instance != null) {
      instance = new LED();
    }
    return instance;
  }
  public LazyCANifier canifier;
  public LightState state = LightState.STEADY;
  public LEDState myLedState = new LEDState(1,1,1);
  public SteadyLEDState mySteadyLED = new SteadyLEDState(myLedState);
  public FlashingLEDState myFlashingLED = new FlashingLEDState(myLedState, myLedState, Constants.kdefaultFlashTime);
  public LED() {
    canifier = new LazyCANifier(Ports.CANIFIER_ID);
    canifier.configFactoryDefault();
  }

  public enum LightState {
    STEADY,
    FLASH,
    OFFSTATE;
  }

  private final Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
        PeriodicIO.rPow = myLedState.red;
        PeriodicIO.gPow = myLedState.green;
        PeriodicIO.bPow = myLedState.blue;
    }

    @Override
    public void onLoop(double timestamp) {
        switch (state) {
            case STEADY:
                break;
            case FLASH:
                myFlashingLED.getCurrentLEDState(myLedState, Timer.getFPGATimestamp()); //This if check is needed within outputTelemetry to update the values that are constantly being alternated between in FlashingLEDState
                PeriodicIO.rPow = myLedState.red;
                PeriodicIO.gPow = myLedState.green;
                PeriodicIO.bPow = myLedState.blue;
                break;
            case OFFSTATE:
                break;
            default:
        }
    }

    @Override
    public void onStop(double timestamp) {
        stop();
    }
};

  public LightState getState() {
    return state;
  }
  public void setState(LightState newState) {
    state = newState;
  }
/*
Deprecated methods
  public void setRedFull() {
    PeriodicIO.rPow = 1;
    PeriodicIO.gPow = 0;
    PeriodicIO.bPow = 0;
  }
  public void setGreenFull() {
    PeriodicIO.rPow = 0;
    PeriodicIO.gPow = 1;
    PeriodicIO.bPow = 0;
  }
  public void setBlueFull() {
    PeriodicIO.rPow = 0;
    PeriodicIO.gPow = 0;
    PeriodicIO.bPow = 1;
  }
*/
  public void setCustomSteady(SteadyLEDState newSteadyState) {
      this.setState(LightState.STEADY); //Set the LightState to steady
      newSteadyState.getCurrentLEDState(myLedState, Timer.getFPGATimestamp()); //myLedState, a class variable LEDState, is given the RGB values of the LEDState within the passed in SteadyLEDState
      PeriodicIO.rPow = myLedState.red; //Set the new RGB values within myLedState to PeriodicIO
      PeriodicIO.gPow = myLedState.green;
      PeriodicIO.bPow = myLedState.blue;
  }

  public void setCustomFlashing(FlashingLEDState newFlashState) {
      this.setState(LightState.FLASH); //Set the LightState to flashing
      myFlashingLED = newFlashState; //NEED TO USE A CLASS VARIABLE HERE to access what is in newFlashState within outputTelemetry. Sets class variable myFlashingLED to the passed in FlashingLEDState
  }

  public void stop() {
    canifier.setLEDOutput(0, LEDChannel.LEDChannelA);
    canifier.setLEDOutput(0, LEDChannel.LEDChannelB);
    canifier.setLEDOutput(0, LEDChannel.LEDChannelC);
  }

  public void writePeriodicOutputs() {
      if(!state.equals(LightState.OFFSTATE)) {
            canifier.setLEDOutput(PeriodicIO.rPow, LEDChannel.LEDChannelA); //Sets the LED output to respective channels using PeriodicIO
            canifier.setLEDOutput(PeriodicIO.gPow, LEDChannel.LEDChannelB);
            canifier.setLEDOutput(PeriodicIO.bPow, LEDChannel.LEDChannelC);
      } else {
             stop();
      }
  }

  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  public static class PeriodicIO {
    public static double rPow;
    public static double gPow;
    public static double bPow;
  }
/*
Code to put in driveWithController()
    //If the state is on, turn it off
    if(operator.getYButton()) {
      if(!led.getState().equals(LightState.OFFSTATE)) {
        led.setState(LightState.OFFSTATE);
      }
    }
    //If "A" pressed, turn on example flashing. If "B" pressed, turn on example steady
    if(operator.getAButton()) {
      led.setCustomFlashing(TimedLEDState.FlashingLEDState.exampleFlashingState);
    } else if (operator.getBButton()) {
      led.setCustomSteady(TimedLEDState.SteadyLEDState.exampleSteadyState);
    }
*/

  @Override
  public void outputTelemetry() {

  }
}