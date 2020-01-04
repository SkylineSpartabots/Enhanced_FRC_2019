/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.states;


public interface TimedLEDState {
    public void getCurrentLEDState(LEDState desiredState, double timestamp);
    /**
    * Creates a FlashingLEDState
    */
    public class FlashingLEDState implements TimedLEDState {
        //Put all flashing led states here
        public static FlashingLEDState exampleFlashingState = new FlashingLEDState(
            LEDState.exampleState, LEDState.off, 0.25);

        private LEDState stateOne = LEDState.exampleState;
        private LEDState stateTwo = new LEDState(0, 0, 0);
        private double duration;

        public FlashingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            this.stateOne.copy(stateOne);
            this.stateTwo.copy(stateTwo);
            this.duration = duration;
        }

        /**
         * This turns the first parameter (LEDState desiredState) into a copy of the current state (based off of a modulus operator that checks which color is currently active) of the FlashingLEDState LEDState.
         * Then use this first parameter as the object to read current RGB values from.
         * Do what you want with these values wherever you call this method.
         * Note that if you want the lights to flash, this method will have to be called within another method that is constantly being updated.
         *
         * @param desiredState The LEDState that will be MODIFIED to become the same as the current state of the FlashingLEDState LEDState
         */
        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if((int) (timestamp/duration) % 2 == 0) {
                desiredState.copy(stateOne);
            } else {
                desiredState.copy(stateTwo);
            }
        }
    }

    /**
     * Creates a SteadyLEDState
     */
    public class SteadyLEDState implements TimedLEDState {
        //put all steady led states here
        public static SteadyLEDState exampleSteadyState = new SteadyLEDState(LEDState.exampleState);

        private LEDState state = new LEDState(0, 0, 0);
        public SteadyLEDState(LEDState state) {
            this.state.copy(state);
        }

        /**
         * This turns the first parameter (LEDState desiredState) into a copy of the current state of the SteadyLEDState LEDState.
         * Then use this first parameter as the object to read current RGB values from.
         * Do what you want with these values wherever you call this method.
         *
         * @param desiredState The LEDState that will be MODIFIED to become the same as the current state of the SteadyLEDState LEDState
         */
        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copy(state);
        }

    }
}
