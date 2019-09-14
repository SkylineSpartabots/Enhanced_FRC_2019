/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drivers.LazySolenoid;
import frc.drivers.LazyTalonSRX;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.auto.SmartDashboardInteractions;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;
import frc.utils.TelemetryUtil;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Intake state machine
 */
public class Intake extends Subsystem {

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }


    private LazyTalonSRX innerIntakeMotor, masterKebab, slaveKebab;
    private List<LazyTalonSRX> motors;
    private LazySolenoid kebabSolenoid;

    private AnalogInput beamBreak;
    private boolean hasCargo;

    public boolean hasCargo() {
        return hasCargo;
    }

    public void setCargoState(boolean state) {
        hasCargo = state;
    }

    private Intake() {
        innerIntakeMotor = new LazyTalonSRX(Ports.INNER_INTAKE_MOTOR);
        masterKebab = new LazyTalonSRX(Ports.RIGHT_KEBAB);
        slaveKebab = new LazyTalonSRX(Ports.LEFT_KEBAB);

        motors = Arrays.asList(innerIntakeMotor, masterKebab, slaveKebab);

        for (LazyTalonSRX motor : motors) {
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configVoltageCompSaturation(12.0);
            motor.enableVoltageCompensation(true);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        }

        innerIntakeMotor.setInverted(true);
        masterKebab.setInverted(true);

        slaveKebab.set(ControlMode.Follower, Ports.RIGHT_KEBAB);
        slaveKebab.setInverted(InvertType.OpposeMaster);

        setCurrentLimits(30);

        kebabSolenoid = new LazySolenoid(Ports.KEBAB_SOLENOID);

        beamBreak = new AnalogInput(Ports.BEAM_BREAK);

    }

    public void setCurrentLimits(int amps) {
        for (LazyTalonSRX motor : motors) {
            motor.configContinuousCurrentLimit(amps);
            motor.configPeakCurrentLimit(amps);
            motor.configPeakCurrentDuration(10);
            motor.enableCurrentLimit(true);
        }
    }

    private void setRampRate(double secondsToMax) {
        for (LazyTalonSRX motor : motors) {
            motor.configOpenloopRamp(secondsToMax, 0);
        }
    }

    /**
     * 
     * @return true if there is a cargo and falso if there is not a cargo
     */
    private boolean isCargoFromSensor() {
        return beamBreak.getValue() < Constants.BEAM_BREAK_THRESHOLD;
    }

    public enum State {
        OFF(0.0, 0.0, false), 
        HOLDING(0.0, 0.0, false), 
        CARGO_PHOBIC(0.0, 0.0, false), 
        IDLE_WITHOUT_KEBABS(0.0, 0.0, false),
        IDLE_WITH_KEBABS(0.0, 0.0, true),
        INTAKE_WITH_KEBABS(0.75, 0.75, true), 
        OUTAKE_WITH_KEBABS(-0.7, -0.7, true),
        INTAKE_WITHOUT_KEBABS(0.75, 0, false),
        OUTAKE_WITHOUT_KEBABS(-0.7, 0, false),
        INTAKE_ELEVATOR_UP(0.35, 0, false),
        OUTAKE_ELEVATOR_UP(-0.6, 0, false);

        public double innerIntakeSpeed = 0.0;
        public double kebabSpeed = 0.0;
        public boolean deployKebabs = false;

        private State(double innerIntakeSpeed, double kebabSpeed, boolean deployKebabs) {
            this.innerIntakeSpeed = innerIntakeSpeed;
            this.kebabSpeed = kebabSpeed;
            this.deployKebabs = deployKebabs;
        }
    }

    private State currentState = State.OFF;
    private boolean stateChanged = false;
    private double beamBreakSensorBeganTimestamp = Double.POSITIVE_INFINITY;

    public State getState() {
        return currentState;
    }

    private synchronized void setState(State newState) {
        if (newState != currentState)
            stateChanged = true;
        currentState = newState;
    }

    public synchronized void setInnerIntakeSpeed(double output) {
        setRampRate(0.0);
        innerIntakeMotor.set(ControlMode.PercentOutput, output);
    }

    public synchronized void setKebabSpeed(double output) {
        setRampRate(0);
        masterKebab.set(ControlMode.PercentOutput, output);
    }

    public synchronized void deployKebabs(boolean deploy) {
        kebabSolenoid.set(deploy);
    }

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            hasCargo = false;
            setState(State.OFF);
            stop();
        }

        @Override
        public void onLoop(double timestamp) {
        if(!SmartDashboardInteractions.cargoSensorOverride.get()) {
            switch(currentState) {
                case OFF:
                    break;
                case IDLE_WITH_KEBABS:
                    break;
                case INTAKE_ELEVATOR_UP:
                    break;
                case OUTAKE_ELEVATOR_UP:
                    break;
                case HOLDING:
                    if(stateChanged) {
                        hasCargo = true;
                    }
                    if(!isCargoFromSensor()) {
                        setInnerIntakeSpeed(0.5);
                    } else {
                        setInnerIntakeSpeed(0);
                    }
                    break;
                case CARGO_PHOBIC:
                    if(stateChanged) {
                        hasCargo = false;
                    }
                    if(isCargoFromSensor()) { 
                        setInnerIntakeSpeed(-0.7);
                    } else {
                        setInnerIntakeSpeed(0);
                    }

                    break;
                default:
                    boolean rawCargo = isCargoFromSensor();
                    if(stateChanged) 
                        hasCargo = false;
                    if(rawCargo) {
                        if(Double.isInfinite(beamBreakSensorBeganTimestamp)) {
                            beamBreakSensorBeganTimestamp = timestamp;
                        } else {
                            if(timestamp - beamBreakSensorBeganTimestamp > 0.3) {
                                hasCargo = true;
                            }
                        }
                    } else if(!Double.isInfinite(beamBreakSensorBeganTimestamp)) {
                        beamBreakSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                    }
                    break;
                }
            } else {
                hasCargo = false;
            }
            
        }

        @Override
        public void onStop(double timestamp) {
            setState(State.OFF);
            stop();
        }

    };

    public void conformToState(State desiredState) {
        setState(desiredState);
        deployKebabs(desiredState.deployKebabs);
        setKebabSpeed(desiredState.kebabSpeed);

        

        if (!(desiredState == State.CARGO_PHOBIC || desiredState == State.HOLDING)) {
            setInnerIntakeSpeed(desiredState.innerIntakeSpeed);
        }

    }

    public Request stateRequest(State desiredState) {
        return new Request() {
            @Override
            public void act() {
                TelemetryUtil.print("Executing", PrintStyle.ERROR);
                conformToState(desiredState);
            }
        };
    }

    public Request waitForCargoRequest() {
        return new Request() {
            @Override
            public void act() {
                conformToState(State.INTAKE_WITH_KEBABS);
            }

            @Override
            public boolean isFinished() {
                return !stateChanged && hasCargo();
            }
        };
    }

    @Override
    public void outputTelemetry() {
        if(Constants.showDebugOutput) {
            SmartDashboard.putBoolean("Raw Cargo", isCargoFromSensor());
            SmartDashboard.putNumber("Inner Intake Current", innerIntakeMotor.getOutputCurrent());
            SmartDashboard.putNumber("Inner Intake Voltage", innerIntakeMotor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Inner Intake Output", innerIntakeMotor.getMotorOutputPercent());
    
            SmartDashboard.putNumber("Right Kebab Current", masterKebab.getOutputCurrent());
            SmartDashboard.putNumber("Right Kebab Voltage", masterKebab.getMotorOutputVoltage());
            SmartDashboard.putNumber("Right Kebab Output", masterKebab.getMotorOutputPercent());
    
            SmartDashboard.putNumber("Left Kebab Current", slaveKebab.getOutputCurrent());
            SmartDashboard.putNumber("Left Kebab Voltage", slaveKebab.getMotorOutputVoltage());
            SmartDashboard.putNumber("Left Kebab Output", slaveKebab.getMotorOutputPercent());
        
            SmartDashboard.putBoolean("Kebab State", kebabSolenoid.get());
        }
        SmartDashboard.putBoolean("Raw Cargo", isCargoFromSensor());
        SmartDashboard.putBoolean("Has Cargo", hasCargo());
        
    }

    @Override
    public void stop() {
        conformToState(State.OFF);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }
}
