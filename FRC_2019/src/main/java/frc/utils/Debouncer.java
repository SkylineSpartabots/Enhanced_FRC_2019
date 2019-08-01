/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import java.util.function.BooleanSupplier;

/**
 * Add your docs here.
 */
public class Debouncer {

    private BooleanSupplier rawInput;
    private int CLOCK_MAX;
    private Boolean deboucedValue = null;

    private boolean prevRawState, rawState;
    private int clockCounter = 0;
    
   /**
    * 
    * @param rawInput Source of the raw "undebounced" input
    * @param CLOCK_MAX Maximum clock count at what point a "debounced" value is recorded
    * @param initDebouncedValue Starting data point for debouncer
    */
    public Debouncer(BooleanSupplier rawInput, int CLOCK_MAX){
        this.rawInput = rawInput;
        this.CLOCK_MAX = CLOCK_MAX;
    }

    public Debouncer() {}

   
    public boolean getAndRecordDebouncedValue() {
        if(rawInput == null) return false;

        rawState = rawInput.getAsBoolean();

        if(prevRawState == rawState) {
            clockCounter++;
            if(clockCounter == CLOCK_MAX){
                deboucedValue = rawState;
                clockCounter = 0;
            }
        } else {
            clockCounter = 0;
        }

        prevRawState = rawState;

        //in the first cycles through the debouncer, when a debounced value hasn't
        //been set, it returns the raw value
        if(deboucedValue == null){
            return rawState;
        } else{
            return deboucedValue;
        }
        
    }

    public void recordValue(boolean input, int MAX_CLOCK_CYCLES) {
        if(prevRawState == input) {
            clockCounter++;
            if(clockCounter == CLOCK_MAX){
                deboucedValue = input;
                clockCounter = 0;
            }
        } else {
            clockCounter = 0;
        }

        prevRawState = input;
    }

    public boolean getDebouncedValue() {
        if(deboucedValue == null){
            return prevRawState;
        } else{
            return deboucedValue;
        }
    }
}
