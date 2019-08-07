/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Relay;
import frc.drivers.LazyTalonSRX;
import frc.robot.Constants;
import frc.robot.Ports;

/**
 * Add your docs here.
 */
public class Climb extends Subsystem {

    private static Climb instance = null;

    public static Climb getInstance() {
        if (instance == null)
            instance = new Climb();
        return instance;
    }

    private LazyTalonSRX master, slave;
    private List<LazyTalonSRX> motors;
    private Relay vacuumMotor;

    public enum ControlState {
        Neutral, Position, OpenLoop, Locked;
    }

    private ControlState state = ControlState.Neutral;

    public ControlState getControlState() {
        return state;
    }

    public void setState(ControlState newState) {
        state = newState;
    }

    private PeriodicIO periodicIO = new PeriodicIO();

    private Climb() {
        vacuumMotor = new Relay(Ports.VACUUM_MOTOR);
        master = new LazyTalonSRX(Ports.RIGHT_CLIMB_MOTOR);
        slave = new LazyTalonSRX(Ports.LEFT_CLIMB_MOTOR);

        motors = Arrays.asList(master, slave);

        slave.follow(master);

        master.follow(master);
        slave.setInverted(InvertType.OpposeMaster);

        for (LazyTalonSRX motor : motors) {
            motor.configVoltageCompSaturation(12.0);
            motor.enableVoltageCompensation(true);
            motor.setNeutralMode(NeutralMode.Brake);
        }

        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        master.setSensorPhase(false); // TODO set true sensor phase

        master.configReverseSoftLimitThreshold(Constants.CLIMB_ENCODER_STARTING_HEIGHT, 10);
        master.configForwardSoftLimitThreshold(
                Constants.CLIMB_ENCODER_STARTING_HEIGHT + inchesToEncUnits(Constants.MAX_CLIMB_HEIGHT));
        master.configReverseSoftLimitEnable(true);
        master.configForwardSoftLimitEnable(true);

        setCurrentLimit(20);

    }

    private void setCurrentLimit(int amps) {
        for (LazyTalonSRX motor : motors) {
            motor.configContinuousCurrentLimit(amps, 10);
            motor.configPeakCurrentLimit(amps, 10);
            motor.configPeakCurrentDuration(10, 10);
            motor.enableCurrentLimit(true);
        }
    }

    private void configForAscent() {
        master.config_kP(0, 0.00, 10);
        master.config_kI(0, 0.00, 10);
        master.config_kD(0, 0.00, 10);
        master.configMotionCruiseVelocity(0, 10);
        master.configMotionAcceleration(0, 10);
        master.configMotionSCurveStrength(0);
        master.selectProfileSlot(0, 0);
    }

    private void configForClimb() {
        master.config_kP(1, 0.00, 10);
        master.config_kI(1, 0.00, 10);
        master.config_kD(1, 0.00, 10);
        master.configMotionCruiseVelocity(0, 10);
        master.configMotionAcceleration(0, 10);
        master.configMotionSCurveStrength(0);
        master.selectProfileSlot(1, 0);
    }

    public boolean isEncoderConnected() {
        int pulseWidthPeriod = master.getSensorCollection().getPulseWidthRiseToRiseUs();
        boolean connected = pulseWidthPeriod != 0;
        if (!connected)
            hasEmergency = true;
        return connected;
    }

    private int inchesToEncUnits(double inches) {
        return (int) (inches * Constants.CLIMB_TICKS_PER_INCH);
    }

    public synchronized boolean isOpenLoop() {
        return state == ControlState.OpenLoop;
    }

    /*public synchronized void setTargetHeight(double heightInFeet) {

        if (isEncoderConnected()) {
            setState(ControlState.Position);
            if (heightInFeet > Constants.MAX_CLIMB_HEIGHT) {
                heightInFeet = Constants.MAX_CLIMB_HEIGHT;
            } else if (heightInFeet < Constants.MIN_CLIMB_HEIGHT) {
                heightInFeet = Constants.MIN_CLIMB_HEIGHT;
            }
            if(heightInFeet > getHeight()) {
                configForAscent();
            } else {
                configForClimb();
            }


        }
    }*/

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    public static class PeriodicIO {
        // Input
        public int position = 0;
        public double velocity = 0.0;
        public double voltage = 0.0;
        public double current = 0.0;

        // Output
        public double demand = 0.0;
    }

}
