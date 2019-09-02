/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class PIDController {
    private double kP, kI, kD;
    private double gainkP = 0, gainkI = 0, gainkD = 0;
    private double setkP, setkI, setkD;
    private double gainScheduleThreshold = 0;
    private double desiredValue;
    protected double prevError;
    private double errorSum;
    protected double finishedRange;
    private double minOutput, maxOutput;
    private int minCycleCount;
    private int currentCycleCount;
    private boolean firstCycle;
    protected boolean debug;
    private double lastTime;
    private double deltaTime;
    private double iRange;
    private DoubleSupplier sensorInput;
    private Debouncer finishedDebouncer;

    public PIDController(double kP, double kI, double kD, double finishedRange, DoubleSupplier sensorInput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.finishedRange = finishedRange;
        this.sensorInput = sensorInput;
        desiredValue = 0.0;
        firstCycle = true;
        maxOutput = 1.0;
        currentCycleCount = 0;
        minCycleCount = 5;
        debug = false;
        minOutput = 0;
        iRange = Double.MAX_VALUE; //this is a default value to always apply I until explicity told not to
        
        finishedDebouncer = new Debouncer();

    }

    public void setConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setGainSchedule(double threshold, double kP, double kI, double kD) {
        gainScheduleThreshold = threshold;
        this.gainkP = kP;
        this.gainkI = kI;
        this.gainkD = kD;
    }

    public void setDesiredValue(double desiredValue) {
        this.desiredValue = desiredValue;
    }

    public void setFinishedRange(double finishedRange) {
        this.finishedRange = finishedRange;
    }

    public void enableDebug() {
        debug = true;
    }

    public void disableDebug() {
        debug = false;
    }

    public void setMinMaxOutput(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public void setMinDoneCycles(int minCycleCount) {
        this.minCycleCount = minCycleCount;
    }

    public void resetErrorSum() {
        errorSum = 0.0;
    }

    public double getDesiredValue() {
        return desiredValue;
    }

    public void setIRange(double iRange) {
        this.iRange = iRange;
    }

    public void reset() {
        firstCycle = true;
        resetErrorSum();
        prevError = 0;
        lastTime = Timer.getFPGATimestamp();
        deltaTime = 20.0;
    }

    public boolean isDone() {
        return finishedDebouncer.getDebouncedValue();
    }

    public double getError() { 
        return desiredValue - sensorInput.getAsDouble();
    }

    public double getOutput() {
        double pVal = 0;
        double iVal = 0;
        double dVal = 0;

        double error = desiredValue - sensorInput.getAsDouble();
        System.out.println(gainkP);
        if(Math.abs(error) < gainScheduleThreshold) {
            TelemetryUtil.print("Gain Scheduling", PrintStyle.ERROR);
            setkP = gainkP;
            setkI = gainkI;
            setkD = gainkD;
        } else {
            setkP = kP;
            setkI = kI;
            setkD = kD;
        }

        if(firstCycle) {
            prevError = error;
            firstCycle = false;
            lastTime = Timer.getFPGATimestamp();
            deltaTime = 20.0;
        } else {
            double currentTime = Timer.getFPGATimestamp();
            deltaTime = currentTime - lastTime;
            lastTime = currentTime;
        }

        deltaTime /= 20;

        //Calculate p value
        pVal = error * setkP;

        //Calculate i value
        if(Math.abs(error) < Math.abs(iRange)) {
            this.errorSum += error*deltaTime;
        } else {
            errorSum = 0;
        }
        iVal = errorSum * setkI;

        //Calculate d value
        double derivative = (error - prevError) / deltaTime;
        dVal = setkD* derivative;

        //overall pid output
        double output  = pVal +iVal + dVal;

        //limit value
        output = Util.limit(output, minOutput, maxOutput);

        finishedDebouncer.recordValue(error <= finishedRange, minCycleCount);
        prevError = error;

        if(debug) {
            SmartDashboard.putNumber("kp", setkP);
            SmartDashboard.putNumber("kI", setkI);
            SmartDashboard.putNumber("kD", setkD);
            SmartDashboard.putNumber("P Out", pVal);
            SmartDashboard.putNumber("I Out", iVal);
            SmartDashboard.putNumber("D Out", dVal);
            SmartDashboard.putNumber("PID Output", output);
            SmartDashboard.putNumber("Error", error);
        }

        return output;
    }


    
}
