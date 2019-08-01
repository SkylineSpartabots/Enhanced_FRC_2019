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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drivers.LazyTalonSRX;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;
import frc.utils.TelemetryUtil;
import frc.utils.Util;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
    private static Elevator instance = null;
    public static Elevator getInstance() {
        if(instance == null)
            instance = new Elevator();
        return instance;
    }


    private LazyTalonSRX master, slave;
    private List<LazyTalonSRX> motors;
    private DigitalInput elevatorLimitSwitch;

    private double manualDriveProportion = Constants.manualElevatorDriveProportion;
    public void setManualDriveProportion(double proportion) {
        manualDriveProportion = proportion;
    }

    private double targetHeight = 0.0;
    public double getTargetHeight() {
        return targetHeight;
    }

    private boolean configuredForAscent;

    private boolean limitsEnabled;
    public boolean isLimitsEnabled() {
        return limitsEnabled;
    }

    public enum ControlState {
        Neutral,
        Position,
        OpenLoop,
        Locked
    }

    private ControlState state = ControlState.Neutral;
    public ControlState getState() {
        return state;
    }

    public void setState(ControlState newState) {
        state = newState;
    }


    private PeriodicIO periodicIO = new PeriodicIO();

    private Elevator() {
        master = new LazyTalonSRX(Ports.RIGHT_ELEVATOR_MOTOR);
        slave = new LazyTalonSRX(Ports.LEFT_ELEVATOR_MOTOR);

        motors = Arrays.asList(master, slave);

        slave.follow(master);

        master.setInverted(false);
        slave.setInverted(InvertType.FollowMaster);

        for(LazyTalonSRX motor : motors) {
            motor.configVoltageCompSaturation(12.0);
            motor.enableVoltageCompensation(true);
            motor.setNeutralMode(NeutralMode.Brake);
        }

        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);    
        master.setSensorPhase(false); //TODO:set true if sensor reads backwards to output

        master.configReverseSoftLimitThreshold(Constants.elevatorEncoderStartingPosition, 10);
        master.configForwardSoftLimitThreshold(Constants.elevatorEncoderStartingPosition + inchesToEncUnits(Constants.maxElevatorHeight), 10);
        master.configReverseSoftLimitEnable(true);
        master.configForwardSoftLimitEnable(true);
        enableLimits(true);

        setCurrentLimit(Constants.elevatorCurrentLimit);

        resetToAbsolutePosition();
        configForAscent();


        elevatorLimitSwitch = new DigitalInput(Ports.ELEVATOR_LIMIT_SWITCH);
    }

    private void setCurrentLimit(int amps) {
        for(LazyTalonSRX motor : motors) {
            motor.configContinuousCurrentLimit(amps, 10);
            motor.configPeakCurrentLimit(amps, 10);
            motor.configPeakCurrentDuration(10, 10);
            motor.enableCurrentLimit(true);
        }
    }

   

    private void configForAscent() {
        master.config_kP(0, 0.016, 10);
        master.config_kI(0, 0.004, 10);
        master.config_kD(0, 0.002, 10);
        /*master.config_kF(0, kF, 10);

        master.config_kP(1, kP, 10);
        master.config_kI(1, kI, 10);
        master.config_kD(1, kD, 10);
        master.config_kF(1, kF, 10);

        master.configMotionCruiseVelocity(sensorUnitsPer100ms);
        master.configMotionAcceleration(sensorUnitsPer100msPerSec);*/
        master.configMotionSCurveStrength(0);

        configuredForAscent = true;
    }

    private void configForDescent() {
        master.configMotionSCurveStrength(4);
        configuredForAscent = false;
    }

    public void enableLimits(boolean enable) {
        master.overrideSoftLimitsEnable(enable);
        limitsEnabled = enable;
    }

    public boolean isEncoderConnected() {
        int pulseWidthPeriod = master.getSensorCollection().getPulseWidthRiseToRiseUs();
        boolean connected = pulseWidthPeriod != 0;
        if(!connected)
            hasEmergency = true;
        return connected;   
    }

    public void setOpenLoop(double input) {
        setState(ControlState.OpenLoop);
        periodicIO.demand = input * manualDriveProportion;
    }

    public boolean isOpenLoop() {
        return getState() == ControlState.OpenLoop || getState() == ControlState.Neutral;
    }

    private int inchesToEncUnits(double inches) {
        return (int) (inches * Constants.elevatorTicksPerInch);
    }

    private double encUnitsToElevatorHeight(double encUnits) {
        return encUnitsToInches(encUnits - Constants.elevatorEncoderStartingPosition);
    }

    private int elevatorHeightToEncUnits(double elevatorHeight) {
        return Constants.elevatorEncoderStartingPosition + inchesToEncUnits(elevatorHeight);
    }

    private double encUnitsToInches(double encUnits){
        return encUnits / Constants.elevatorTicksPerInch;
    }

    public double getHeight() {
        return encUnitsToElevatorHeight(periodicIO.position);
    }

    public double getVelocityFeetPerSec() {
        return encUnitsToInches(periodicIO.velocity) * 10.0;
    }

    public synchronized void setTargetHeight(double heightInFeet) {
        setState(ControlState.Position);
        if(heightInFeet > Constants.maxElevatorHeight) {
            heightInFeet = Constants.maxElevatorHeight;
        } else if(heightInFeet < Constants.minElevatorHeight) {
            heightInFeet = Constants.minElevatorHeight;
        }

        if(isEncoderConnected()) {
            if(heightInFeet > getHeight()) {
                master.selectProfileSlot(0, 0);
                configForAscent();
            } else {
                master.selectProfileSlot(1, 0);
                configForDescent();
            }
            targetHeight = heightInFeet;
            periodicIO.demand = elevatorHeightToEncUnits(heightInFeet);
            onTarget = false;
            startTime = Timer.getFPGATimestamp();
        } else {
            TelemetryUtil.print("elevator encoder is not detected", PrintStyle.ERROR);
            stop();
        }
    }

    private boolean onTarget = false;
    private double startTime = 0.0;

    public boolean hasReachedTargetHeight() {
        if(master.getControlMode() == ControlMode.MotionMagic) {
            if(Math.abs(targetHeight - getHeight()) <= Constants.elevatorHeightTolerance) {
                if(!onTarget)
                    onTarget = true;
                return onTarget;
            }
        }
        return false;
    }

    public synchronized void lockHeight() {
        setState(ControlState.Locked);
        if(isEncoderConnected()) {
            targetHeight = getHeight();
            periodicIO.demand = periodicIO.position;    
        } else {
            TelemetryUtil.print("elevator encoder is not detected", PrintStyle.ERROR);
            stop();
        }
    }

    public void resetToAbsolutePosition() {
        int absolutePosition = (int) Util.boundToScope(0, 4096, master.getSensorCollection().getPulseWidthPosition());
        if(encUnitsToElevatorHeight(absolutePosition) > Constants.maxElevatorInitialHeight) {
            absolutePosition -= 4096;
        } else if(encUnitsToElevatorHeight(absolutePosition) < Constants.minElevatorInitialHeight) {
            absolutePosition += 4096;
        }
        double height = encUnitsToElevatorHeight(absolutePosition);
        if(height > Constants.maxElevatorInitialHeight || height < Constants.minElevatorHeight) {
            TelemetryUtil.print("Elevator height is out of bounds", PrintStyle.ERROR);
            hasEmergency = true;
        }
        master.setSelectedSensorPosition(absolutePosition, 0, 10);
     }

    public Request openLoopRequest(double input) {
        return new Request() {
            @Override
            public void act() {
                setOpenLoop(input);
            }
        };
    }

    public Request heightRequest(double heightInFeet) {
        return new Request(){
        
            @Override
            public void act() {
                setTargetHeight(heightInFeet);
            }

            @Override
            public boolean isFinished() {
                return hasReachedTargetHeight() || isOpenLoop();
            }
        };
    }

    public Request lockHeightRequest() {
        return new Request(){
            @Override
            public void act() {
                lockHeight();
            }
        };
    }


    private boolean limitSwitchChangedState = false;
    
    private final Loop loop = new Loop(){
    
        @Override
        public void onStart(double timestamp) {
            
        }
    
        @Override
        public void onLoop(double timestamp) {
            if(elevatorLimitSwitch.get()) {
                if(!limitSwitchChangedState) {
                    resetToAbsolutePosition();
                    limitSwitchChangedState = true;
                }
            } else {
                limitSwitchChangedState = false;
            }
        }

        @Override
        public void onStop(double timestamp) {
            
        }
    };


    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.position = master.getSelectedSensorPosition();

        if(Constants.showDebugOutput) {
            periodicIO.velocity = master.getSelectedSensorVelocity(0);
            periodicIO.voltage = master.getMotorOutputVoltage();
            periodicIO.current = master.getOutputCurrent();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if(getState() == ControlState.Position || getState() == ControlState.Locked) {
            master.set(ControlMode.MotionMagic, periodicIO.demand);
        } else {
            master.set(ControlMode.PercentOutput, periodicIO.demand);
        }
    }

    @Override
    public void zeroSensors() {
        resetToAbsolutePosition();
    }


    @Override
    public void outputTelemetry() {
        if(Constants.showDebugOutput) {
            SmartDashboard.putNumber("Right Elevator Current", master.getOutputCurrent());
            SmartDashboard.putNumber("Left Elevator Current", slave.getOutputCurrent());
            SmartDashboard.putNumber("Right Elevator Voltage", master.getMotorOutputVoltage());
            SmartDashboard.putNumber("Left Elevator Voltage", slave.getMotorOutputVoltage());

            SmartDashboard.putNumber("Elevator Output", master.getMotorOutputPercent());

            SmartDashboard.putNumber("Elevator Height Graph", getHeight());
            SmartDashboard.putNumber("Elevator Pulse Width Position", master.getSensorCollection().getPulseWidthPosition());
            SmartDashboard.putNumber("Elevator Encoder", periodicIO.position);
            SmartDashboard.putNumber("Elevator Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("Elevator Error", master.getClosedLoopTarget(0));

            SmartDashboard.putBoolean("Elevator Limit Switch", elevatorLimitSwitch.get());

            if(master.getControlMode() == ControlMode.MotionMagic) {
                SmartDashboard.putNumber("Elevator Setpoint", master.getClosedLoopTarget(0));
            }
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }
    
    public static class PeriodicIO {
        //Input
        public int position = 0;
        public double velocity = 0.0;
        public double voltage = 0.0;
        public double current = 0.0;

        //Ouput
        public double demand = 0.0;
    }

   


}
