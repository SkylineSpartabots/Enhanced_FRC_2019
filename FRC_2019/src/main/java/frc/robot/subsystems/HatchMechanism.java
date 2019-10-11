/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drivers.LazySolenoid;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.auto.SmartDashboardInteractions;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;

/**
 * Add your docs here.
 */
public class HatchMechanism extends Subsystem {

    private static HatchMechanism instance = null;

    public static HatchMechanism getInstance() {
        if (instance == null)
            instance = new HatchMechanism();
        return instance;
    }

    private LazySolenoid slider, fingers, jacks;
    private AnalogInput hatchDistanceSensor;

    private HatchMechanism() {
        slider = new LazySolenoid(Ports.SLIDER_SOLENOID);
        fingers = new LazySolenoid(Ports.FINGERS_SOLENOID);
        jacks = new LazySolenoid(Ports.JACKS_SOLENOID);
        hatchDistanceSensor = new AnalogInput(Ports.HATCH_DISTANCE_SENSOR);
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
        if (newState != currentState)
            stateChanged = true;
        currentState = newState;
    }

    private boolean stateChanged = false;
    private double distanceSensorBeganTimestamp = Double.POSITIVE_INFINITY;
    private double scoringStateBeganTimestamp = 0;
    private boolean hasHatch = false;

    public boolean hasHatch() {
        return hasHatch;
    }

    public void conformToState(State desiredState) {
        fingers.set(desiredState.fingersExtended);
        slider.set(desiredState.sliderExtended);
        if(desiredState != State.SCORING) {
            jacks.set(desiredState.jacksExtended);
        }
        setState(desiredState);
    }

    private boolean hasHatchFromSensor() {
        return hatchDistanceSensor.getValue() > Constants.HATCH_DISTANCE_THRESHOLD;
    }

    public Request stateRequest(State newState) {
        return new Request() {

            @Override
            public void act() {
                conformToState(newState);
            }
        };
    }

    public Request waitForHatchRequest() {
        return new Request() {

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

            if (!SmartDashboardInteractions.hatchSensorOverride.get()) {
                /*if (currentState == State.STOWED) {
                    if (stateChanged) {
                        hasHatch = hasHatchFromSensor();
                    }
                } else {*/
                    if (hasHatchFromSensor()) {
                        //System.out.println("raw has hatch from sensor " + hasHatchFromSensor());
                        if (Double.isInfinite(distanceSensorBeganTimestamp)) {
                            distanceSensorBeganTimestamp = timestamp;
                        } else {
                            if (timestamp - distanceSensorBeganTimestamp >= 0.1) {
                                hasHatch = true;
                            }
                        }
                    } else if (!Double.isInfinite(distanceSensorBeganTimestamp)) {
                        distanceSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                        hasHatch = false;
                    }
                    if (currentState == State.RECIEVING && hasHatch) {
                        conformToState(State.STOWED);
                    }
                //}
            } else {
                hasHatch = false;
            }
            
            if(currentState == State.SCORING) {
                if(stateChanged) {
                    scoringStateBeganTimestamp = timestamp;
                }
                if(timestamp - scoringStateBeganTimestamp > 0) {
                    jacks.set(true);
                } else {
                    jacks.set(false);
                }
            }

            stateChanged = false;
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
        SmartDashboard.putBoolean("Has hatch", hasHatch());
        SmartDashboard.putNumber("Raw Distance Sensor", hatchDistanceSensor.getValue());
        SmartDashboard.putBoolean("Fingers State", fingers.get());
        if (Constants.showDebugOutput) {
            SmartDashboard.putBoolean("Fingers State", fingers.get());
            SmartDashboard.putBoolean("Slider State", slider.get());
            SmartDashboard.putBoolean("Jacks State", jacks.get());
            SmartDashboard.putNumber("Raw Distance Sensor", hatchDistanceSensor.getValue());
        }
    }

    @Override
    public void stop() {

    }
}
