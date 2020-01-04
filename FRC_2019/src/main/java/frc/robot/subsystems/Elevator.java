/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;

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
import frc.utils.PIDController;
import frc.utils.TelemetryUtil;
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
    private PIDController elevatorPID;

    private double manualDriveProportion = Constants.manualElevatorDriveProportion;

    public void setManualDriveProportion(double proportion) {
        manualDriveProportion = proportion;
    }

    private double targetHeight = 0.0;

    public double getTargetHeight() {
        return targetHeight;
    }

    private boolean configuredForAscent = false;

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

        master.setInverted(false);
        slave.setInverted(InvertType.OpposeMaster);

        for (LazyTalonSRX motor : motors) {
            motor.configVoltageCompSaturation(12.0);
            motor.enableVoltageCompensation(true);
            motor.setNeutralMode(NeutralMode.Brake);
        }

        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        master.setSensorPhase(true);

        master.configReverseSoftLimitThreshold(Constants.elevatorEncoderStartingPosition);
        master.configForwardSoftLimitThreshold(4700);
        master.configReverseSoftLimitEnable(true);
        master.configForwardSoftLimitEnable(true);

        setCurrentLimit(Constants.elevatorCurrentLimit);

        resetToAbsolutePosition();

        DoubleSupplier elevetorEncoderSupplier = () -> periodicIO.position;
        elevatorPID = new PIDController(0.0005, 0.014, 0.0, 10, elevetorEncoderSupplier);

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
     * previous elevator pid kp = 0.016 ki = 0.004 kd = 0.002
     * 
     * try kf = 1023/velocity try accel = 3*velocity
     */

    private void configForAscent() {
        if (!configuredForAscent) {
            elevatorPID.setConstants(0.0005, 0.014, 0.0);
            elevatorPID.setMinMaxOutput(-0.3, 0.73);
            elevatorPID.setIRange(1000);
            TelemetryUtil.print("Config for ascent", PrintStyle.INFO);
            configuredForAscent = true;
        }
    }

    private void configForDescent() {
        if (configuredForAscent) {
            elevatorPID.setConstants(0.02, 0, 0);
            elevatorPID.setMinMaxOutput(-0.3, 0.1);
            TelemetryUtil.print("Config for descent", PrintStyle.INFO);
            configuredForAscent = false;
        }
    }

    public void enableLimits(boolean enable) {
        master.overrideSoftLimitsEnable(enable);
        limitsEnabled = enable;
    }

    public boolean getLimitSwitch() {
        return !elevatorLimitSwitch.get();
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

        if (heightInFeet > Constants.maxElevatorHeight) {
            heightInFeet = Constants.maxElevatorHeight;
        } else if (heightInFeet < Constants.minElevatorHeight) {
            heightInFeet = Constants.minElevatorHeight;
        }
        if (!SmartDashboardInteractions.elevatorEncoderOverride.get()) {
            setState(ControlState.Position);
            if (heightInFeet > getHeight()) {
                configForAscent();
            } else {
                configForDescent();
            }
            targetHeight = heightInFeet;
            periodicIO.demand = elevatorHeightToEncUnits(heightInFeet);
            elevatorPID.reset();
            elevatorPID.setDesiredValue(periodicIO.demand);
            onTarget = false;
        } else {
            TelemetryUtil.print("closed loop elevator controls disabled", PrintStyle.ERROR);
            stop();
        }
    }

    private boolean onTarget = false;

    public boolean hasReachedTargetHeight() {
        if (Math.abs(targetHeight - getHeight()) <= Constants.elevatorHeightTolerance) {
            if (!onTarget)
                onTarget = true;
            return onTarget;
        }
        return false;
    }

    public synchronized void lockHeight() {
        setState(ControlState.Locked);
        periodicIO.demand = periodicIO.position;
    }

    private double getLockPower() {
        if (SmartDashboardInteractions.elevatorEncoderOverride.get())
            return 0.15;
        return 0.15 + (0.0004 * (periodicIO.demand - periodicIO.position));
    }

    public double getAntiTipCoeffecient() {
        if (SmartDashboardInteractions.antiTipOverride.get()) {
            return 1;
        }
        return Math.abs(1 - (periodicIO.position * -0.00010725));
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

    public Request logVelocity(String filePath) {
        return new Request() {

            @Override
            public void act() {
                java.io.File f = new java.io.File(filePath);
                PrintWriter pw = null;
                System.out.println("logging");
                try {
                    pw = new PrintWriter(f);
                } catch (FileNotFoundException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                pw.write("at end of time, velocity: " + getVelocityFeetPerSec());
                pw.write("height: " + getHeight());
                System.out.println("||||||||||||||||||||||||||||||||||||||||||||||");
                System.out.println("at end of time, velocity: " + master.getSelectedSensorVelocity());
                System.out.println("height: " + getHeight());
                System.out.println("||||||||||||||||||||||||||||||||||||||||||||||");
                
            }
        };
    }

    public Request timedPowerSet(double power, double time) {
        return new Request(){
            double startTime = 0;
            @Override
            public void act() {
                System.out.println("timed power called with power: " + power);
                startTime = Timer.getFPGATimestamp();
                periodicIO.demand = power;
            }
            @Override
            public boolean isFinished() {
                System.out.println("at end of time, velocity: " + master.getSelectedSensorVelocity());
                return (Timer.getFPGATimestamp() - startTime) >= time;
            }
        };
    }

    public boolean isElevatorUp() {
        return isElevatorUp;
    }

    private boolean isElevatorUp = false;
    private double elevatorUpBeganTimestamp = 0;

    private final Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            resetToAbsolutePosition();
            setState(ControlState.OpenLoop);
            periodicIO.demand = 0;
            targetHeight = 0;
        }

        @Override
        public void onLoop(double timestamp) {
            if (!SmartDashboardInteractions.elevatorLimitSwitchOverride.get()) {
                if (getLimitSwitch() && periodicIO.position != 0) {
                    resetToAbsolutePosition();
                }
            }

            if (SmartDashboardInteractions.elevatorEncoderOverride.wasOverriden()) {
                enableLimits(false);
            }

            if (!SmartDashboardInteractions.elevatorEncoderOverride.get()) {
                if (getHeight() > 2) {
                    if (Double.isInfinite(elevatorUpBeganTimestamp)) {
                        elevatorUpBeganTimestamp = timestamp;
                    } else {
                        if (timestamp - elevatorUpBeganTimestamp > 0.2) {
                            isElevatorUp = true;
                        }
                    }
                } else if (!Double.isInfinite(elevatorUpBeganTimestamp)) {
                    elevatorUpBeganTimestamp = Double.POSITIVE_INFINITY;
                    isElevatorUp = false;
                }
            } else {
                isElevatorUp = false;
            }

        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    @Override
    public synchronized void readPeriodicInputs() {

        if (!SmartDashboardInteractions.elevatorEncoderOverride.get()) {
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
        if (periodicIO.demand <= 0 && periodicIO.position <= 250 && !SmartDashboardInteractions.elevatorEncoderOverride.get()) {
            if (getLimitSwitch()) {
                master.set(ControlMode.PercentOutput, 0);
            } else if (!SmartDashboardInteractions.elevatorLimitSwitchOverride.get()) {
                master.set(ControlMode.PercentOutput, -0.2);
                TelemetryUtil.print("ELEVATOR POWER DOWN TO LIMIT SWITCH", PrintStyle.WARNING);
            }
        } else if (getState() == ControlState.Position && !SmartDashboardInteractions.elevatorEncoderOverride.get()) {
            master.set(ControlMode.PercentOutput, elevatorPID.getOutput());
        } else if (getState() == ControlState.Locked) {
            master.set(ControlMode.PercentOutput, getLockPower());
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
            SmartDashboard.putNumber("Slave Output", slave.getMotorOutputPercent());
            SmartDashboard.putNumber("Elevator Output", master.getMotorOutputPercent());
            SmartDashboard.putNumber("Right Elevator Current", master.getOutputCurrent());
            SmartDashboard.putNumber("Left Elevator Current", slave.getOutputCurrent());
            SmartDashboard.putNumber("Right Elevator Voltage", master.getMotorOutputVoltage());
            SmartDashboard.putNumber("Left Elevator Voltage", slave.getMotorOutputVoltage());
            SmartDashboard.putNumber("Demand", periodicIO.demand);
            SmartDashboard.putNumber("Desired Position", elevatorPID.getDesiredValue());
            SmartDashboard.putNumber("Elevator Encoder", periodicIO.position);
            elevatorPID.enableDebug();
        }

        
    
        SmartDashboard.putNumber("Elevator Height", getHeight());
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
