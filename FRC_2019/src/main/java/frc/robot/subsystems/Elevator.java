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
import frc.robot.auto.SmartDashboardInteractions;
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
        if (instance == null)
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
        Neutral, Position, OpenLoop, Locked
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
        master = new LazyTalonSRX(Ports.LEFT_ELEVATOR_MOTOR);
        slave = new LazyTalonSRX(Ports.RIGHT_ELEVATOR_MOTOR);

        motors = Arrays.asList(master, slave);

        slave.set(ControlMode.Follower, Ports.LEFT_ELEVATOR_MOTOR);

        master.setInverted(true);
        slave.setInverted(InvertType.OpposeMaster);

        for (LazyTalonSRX motor : motors) {
            motor.configVoltageCompSaturation(12.0);
            motor.enableVoltageCompensation(true);
            motor.setNeutralMode(NeutralMode.Brake);
        }

        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        master.setSensorPhase(false);

        master.configReverseSoftLimitThreshold(Constants.elevatorEncoderStartingPosition);
        //master.configForwardSoftLimitThreshold(
        //        Constants.elevatorEncoderStartingPosition + inchesToEncUnits(Constants.maxElevatorHeight), 10);
        master.configForwardSoftLimitThreshold(500);
        master.configReverseSoftLimitEnable(true);
        master.configForwardSoftLimitEnable(true);

        setCurrentLimit(Constants.elevatorCurrentLimit);

        resetToAbsolutePosition();
        configForAscent();

        elevatorLimitSwitch = new DigitalInput(Ports.ELEVATOR_LIMIT_SWITCH);
    
    }

    private void setCurrentLimit(int amps) {
        for (LazyTalonSRX motor : motors) {
            motor.configContinuousCurrentLimit(amps);
            motor.configPeakCurrentLimit(amps);
            motor.configPeakCurrentDuration(10);
            motor.enableCurrentLimit(true);
        }
    }

    /**
     * previous elevator pid
     *  kp = 0.016
     *  ki = 0.004
     *  kd = 0.002
     * 
     *  try kf = 1023/velocity
     *  try accel = 3*velocity
     */

    private void configForAscent() {
        //if(!configuredForAscent) {
            master.config_kP(0, SmartDashboardInteractions.elevatorAscentConstants.getkP());
            master.config_kI(0, SmartDashboardInteractions.elevatorAscentConstants.getkI());
            master.config_kD(0, SmartDashboardInteractions.elevatorAscentConstants.getkD());
            master.config_kF(0, SmartDashboardInteractions.elevatorAscentConstants.getkF());
            master.configMotionCruiseVelocity(SmartDashboardInteractions.elevatorAscentConstants.getVelocity());
            master.configMotionAcceleration(SmartDashboardInteractions.elevatorAscentConstants.getAcceleration());
            master.configMotionSCurveStrength(0);
            TelemetryUtil.print("Config for ascent", PrintStyle.INFO);
            configuredForAscent = true;
        //} 
    }

    private void configForDescent() {
        //if(configuredForAscent) {
            master.config_kI(0, SmartDashboardInteractions.elevatorDescentConstants.getkI());
            master.config_kD(0, SmartDashboardInteractions.elevatorDescentConstants.getkD());
            master.config_kF(0, SmartDashboardInteractions.elevatorDescentConstants.getkF());
            master.config_kP(0, SmartDashboardInteractions.elevatorDescentConstants.getkP());
            master.configMotionCruiseVelocity(SmartDashboardInteractions.elevatorDescentConstants.getVelocity());
            master.configMotionAcceleration(SmartDashboardInteractions.elevatorDescentConstants.getAcceleration());
            master.configMotionSCurveStrength(4);
            TelemetryUtil.print("Config for ascent", PrintStyle.INFO);
            configuredForAscent = false;
        //} 
    }

    public void enableLimits(boolean enable) {
        master.overrideSoftLimitsEnable(enable);
        limitsEnabled = enable;
    }
    

    public boolean getLimitSwitch() {
        return !elevatorLimitSwitch.get();
    }

    public boolean isEncoderConnected() {
        int pulseWidthPeriod = master.getSensorCollection().getPulseWidthRiseToRiseUs();
        boolean connected = pulseWidthPeriod != 0;
        if (!connected)
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

    private double encUnitsToInches(double encUnits) {
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
        if (heightInFeet > Constants.maxElevatorHeight) {
            heightInFeet = Constants.maxElevatorHeight;
        } else if (heightInFeet < Constants.minElevatorHeight) {
            heightInFeet = Constants.minElevatorHeight;
        }

        if (isEncoderConnected() && !SmartDashboardInteractions.elevatorEncoderOverride.get()) {
            if (heightInFeet > getHeight()) {
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
            TelemetryUtil.print("closed loop elevator controls disabled", PrintStyle.ERROR);
            stop();
        }
    }

    private boolean onTarget = false;
    private double startTime = 0.0;

    public boolean hasReachedTargetHeight() {
        if (master.getControlMode() == ControlMode.MotionMagic) {
            if (Math.abs(targetHeight - getHeight()) <= Constants.elevatorHeightTolerance) {
                if (!onTarget)
                    onTarget = true;
                return onTarget;
            }
        }
        return false;
    }

    public synchronized void lockHeight() {
        setState(ControlState.Locked);
        if (isEncoderConnected() && !SmartDashboardInteractions.elevatorEncoderOverride.get()) {
            targetHeight = getHeight();
            periodicIO.demand = periodicIO.position;
        } else {
            TelemetryUtil.print("closed loop elevator controls disabled", PrintStyle.ERROR);
            stop();
        }
    }

    public void getAntiTipCoeffecient() {
        
    }

    public void resetToAbsolutePosition() {
        master.setSelectedSensorPosition(0, 0, 10);
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
        return new Request() {

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
        return new Request() {
            @Override
            public void act() {
                lockHeight();
            }
        };
    }

    private boolean limitSwitchChangedState = false;

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            resetToAbsolutePosition();
        }

        @Override
        public void onLoop(double timestamp) {
            if (!SmartDashboardInteractions.elevatorLimitSwitchOverride.get()) {
                if (getLimitSwitch() && periodicIO.position != 0) {
                    resetToAbsolutePosition();
                }
            }

            /*if(SmartDashboardInteractions.elevatorEncoderOverride.wasOverriden()) {
                enableLimits(false);
            }*/
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    @Override
    public synchronized void readPeriodicInputs() {

        if(!SmartDashboardInteractions.elevatorEncoderOverride.get()) {
            periodicIO.position = master.getSelectedSensorPosition();
        } else {
            periodicIO.position = 0;
        }
        

        if (Constants.showDebugOutput) {
            periodicIO.velocity = master.getSelectedSensorVelocity(0);
            periodicIO.voltage = master.getMotorOutputVoltage();
            periodicIO.current = master.getOutputCurrent();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if(periodicIO.demand <= 0 && periodicIO.position <= 70) {
            if(getLimitSwitch()) {
                master.set(ControlMode.PercentOutput, 0);
            } else {
                master.set(ControlMode.PercentOutput, -0.2);
            }
        } else if (getState() == ControlState.Position || getState() == ControlState.Locked) { 
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
        if (Constants.showDebugOutput) {
            SmartDashboard.putNumber("Right Elevator Current", master.getOutputCurrent());
            SmartDashboard.putNumber("Left Elevator Current", slave.getOutputCurrent());
            SmartDashboard.putNumber("Right Elevator Voltage", master.getMotorOutputVoltage());
            SmartDashboard.putNumber("Left Elevator Voltage", slave.getMotorOutputVoltage());

            SmartDashboard.putNumber("Elevator Output", master.getMotorOutputPercent());

            SmartDashboard.putNumber("Elevator Height Graph", getHeight());
            SmartDashboard.putNumber("Elevator Pulse Width Position",
                    master.getSensorCollection().getPulseWidthPosition());
            SmartDashboard.putNumber("Elevator Encoder", periodicIO.position);
            SmartDashboard.putNumber("Elevator Velocity", periodicIO.velocity);
            SmartDashboard.putNumber("Elevator Error", master.getClosedLoopTarget(0));

            SmartDashboard.putBoolean("Elevator Limit Switch", getLimitSwitch());

            if (master.getControlMode() == ControlMode.MotionMagic) {
                SmartDashboard.putNumber("Elevator Setpoint", master.getClosedLoopTarget(0));
            }
        }

        SmartDashboard.putNumber("Slave Output", slave.getMotorOutputPercent());
        SmartDashboard.putNumber("Elevator Output", master.getMotorOutputPercent());
        SmartDashboard.putNumber("Right Elevator Current", master.getOutputCurrent());
            SmartDashboard.putNumber("Left Elevator Current", slave.getOutputCurrent());
            SmartDashboard.putNumber("Right Elevator Voltage", master.getMotorOutputVoltage());
            SmartDashboard.putNumber("Left Elevator Voltage", slave.getMotorOutputVoltage());
        SmartDashboard.putNumber("Elevator Encoder", periodicIO.position);
        SmartDashboard.putBoolean("Elevator Limit Switch", getLimitSwitch());
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
        // Input
        public int position = 0;
        public double velocity = 0.0;
        public double voltage = 0.0;
        public double current = 0.0;

        // Ouput
        public double demand = 0.0;
    }

}
