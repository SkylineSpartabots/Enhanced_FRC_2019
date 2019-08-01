/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drivers.LazySolenoid;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;

/**
 * Add your docs here.
 */
public class HatchMechanism extends Subsystem {

    private static HatchMechanism instance = null;
    public static HatchMechanism getInstance() {
        if(instance == null) 
            instance = new HatchMechanism();
        return instance;
    }

    private LazySolenoid slider, fingers, jacks;
    private DigitalInput hatchLimitSwitch;


    private HatchMechanism() {
        slider = new LazySolenoid(Ports.SLIDER_SOLENOID);
        fingers = new LazySolenoid(Ports.FINGERS_SOLENOID);
        jacks = new LazySolenoid(Ports.JACKS_SOLENOID);
        hatchLimitSwitch = new DigitalInput(Ports.HATCH_LIMIT_SWITCH);
    }

    public enum State {
        SCORING(true, true, true),
        RECIEVING(true, true, false),
        STOWED(false, false, false),
        FINGERS_STOWED_EXTENDED(false, true, false),
        FINGERS_EXTENDED_RETRACTED(true, false, false);
        
        public boolean fingersExtended;
        public boolean sliderExtended;
        public boolean jacksExtended;

        private State(boolean fingersExtended, boolean sliderExtended, boolean jacksExtended) {
            this.fingersExtended = fingersExtended;
            this.sliderExtended = sliderExtended;
            this.jacksExtended = jacksExtended;
        }
    }

    private State currentState = State.STOWED;
    public State getState() {
        return currentState;
    }

    private synchronized void setState(State newState) {
        if(newState != currentState) 
            stateChanged = true;
        currentState = newState;
    }


    private boolean stateChanged = false;
    private double limitSwitchBeganTimestamp = Double.POSITIVE_INFINITY;

    private boolean hasHatch = false;
    public boolean hasHatch() {
        return hasHatch;
    }

    public void conformToState(State desiredState) {
        fingers.set(desiredState.fingersExtended);
        slider.set(desiredState.sliderExtended);
        jacks.set(desiredState.jacksExtended);
        setState(desiredState);
    }

    public Request stateRequest(State newState) {
        return new Request(){
        
            @Override
            public void act() {
                conformToState(newState);
            }
        };
    }

    public Request waitForHatchRequest() {
        return new Request(){
        
            @Override
            public void act() {
                
            }

            @Override
            public boolean isFinished() {
                return hasHatch() && !stateChanged;
            }
        };
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            switch(currentState) {
                case STOWED:
                    break;
                case FINGERS_STOWED_EXTENDED:
                    break;
                default:
                    if(hatchLimitSwitch.get()) {
                        if(Double.isInfinite(limitSwitchBeganTimestamp)) {
                            limitSwitchBeganTimestamp = timestamp;  
                        } else {
                            if(timestamp - limitSwitchBeganTimestamp >= 0.0) {
                                hasHatch = true;
                            }
                        }
                    } else if (Double.isFinite(limitSwitchBeganTimestamp)) {
                        limitSwitchBeganTimestamp = Double.POSITIVE_INFINITY;
                        hasHatch = false;
                    }
                    if(currentState == State.RECIEVING && hasHatch) {
                        conformToState(State.STOWED);
                    }
                    break;
            }


            if(stateChanged) {
                stateChanged = false;
            }

        }

        @Override
        public void onStop(double timestamp) {
            conformToState(State.STOWED);
        }

    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void outputTelemetry() {
        if(Constants.showDebugOutput) {
            SmartDashboard.putBoolean("Fingers State", fingers.get());
            SmartDashboard.putBoolean("Slider State", slider.get());
            SmartDashboard.putBoolean("Jacks State", jacks.get());
            SmartDashboard.putBoolean("Raw Hatch Switch", hatchLimitSwitch.get());
        }
    }

    @Override
    public void stop() {

    }
}
