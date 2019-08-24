/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drivers.LazyTalonSRX;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.RobotState;
import frc.robot.loops.Limelight;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Pose2dWithCurvature;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.trajectory.TrajectoryIterator;
import frc.team254.lib.trajectory.timing.TimedState;
import frc.team254.planners.DriveMotionPlanner;
import frc.utils.DriveSignal;
import frc.utils.Navx;
import frc.utils.PIDController;
import frc.utils.TelemetryUtil;
import frc.utils.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {

    private static Drivetrain instance = null;
    public static Drivetrain getInstance() {
        if(instance == null)
            instance = new Drivetrain();
        return instance;
    }

    public static final double DRIVE_ENCODER_PPR = 1000;
    public static final double DESIRED_TARGET_AREA = 0; //TODO
    public static final double VISION_THRESHOLD = 0; //TODO

    private DriveControlState state;
    private PeriodicIO periodicIO;

    private DriveMotionPlanner motionPlanner;
    private Rotation2d gyroOffset = Rotation2d.identity();
    private boolean overrideTrajectory = false;

    private PIDController visionDrivePID;
    private PIDController visionTurnPID;

    private LazyTalonSRX rightMaster, rightSlaveA, rightSlaveB;
    private LazyTalonSRX leftMaster, leftSlaveA, leftSlaveB;
    private List<LazyTalonSRX> motors, masters, slaves;

    private boolean isBrakeMode;

    private boolean stateChanged;


    private Drivetrain() {
        leftMaster = new LazyTalonSRX(Ports.DRIVE_LEFT_CENTER);
        leftSlaveA = new LazyTalonSRX(Ports.DRIVE_LEFT_FRONT);
        leftSlaveB = new LazyTalonSRX(Ports.DRIVE_LEFT_BACK);

        rightMaster = new LazyTalonSRX(Ports.DRIVE_RIGHT_CENTER);
        rightSlaveA = new LazyTalonSRX(Ports.DRIVE_RIGHT_FRONT);
        rightSlaveB = new LazyTalonSRX(Ports.DRIVE_RIGHT_BACK);

        motors = Arrays.asList(rightMaster, rightSlaveA, rightSlaveB, leftMaster, leftSlaveA, leftSlaveB);
        masters = Arrays.asList(rightMaster, leftMaster);
        slaves = Arrays.asList(rightSlaveA, rightSlaveB, leftSlaveA, leftSlaveB);

        leftSlaveA.set(ControlMode.Follower, Ports.DRIVE_LEFT_CENTER);
        leftSlaveB.set(ControlMode.Follower, Ports.DRIVE_LEFT_CENTER);
        rightSlaveA.set(ControlMode.Follower, Ports.DRIVE_RIGHT_CENTER);
        rightSlaveB.set(ControlMode.Follower, Ports.DRIVE_RIGHT_CENTER);

        leftMaster.setInverted(InvertType.InvertMotorOutput);
        rightMaster.setInverted(InvertType.None);
        slaves.forEach((s) -> s.setInverted(InvertType.FollowMaster));

        for(LazyTalonSRX motor : motors) {
            motor.configVoltageCompSaturation(12.0);
            motor.enableVoltageCompensation(true);
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configNominalOutputForward(1.0/12.0);
            motor.configNominalOutputReverse(-1.0/12.0);
            motor.configOpenloopRamp(0.2);
            motor.configClosedloopRamp(0.0);
        }

        for(LazyTalonSRX master : masters) {
            master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            master.setSelectedSensorPosition(0, 0, 50);
            //master.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
            master.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 10);
            master.configVelocityMeasurementWindow(32, 10);
            master.setSensorPhase(false); //TODO: set sensor direction
            //TODO: add values
            /*//settings for motion profiling
            master.selectProfileSlot(0, 0);
            master.config_kP(0, kP);
            master.config_kI(0, kI);
            master.config_kD(0, kD);
            master.config_kF(0, kF);
            master.configMotionCruiseVelocity(sensorUnitsPer100ms);
            master.configMotionAcceleration(sensorUnitsPer100msPerSec);
            //settings for velocity mode
            master.config_kP(1, kP);
            master.config_kI(1, kI);
            master.config_kD(1, kD);
            master.config_kF(1, kF);*/
        }

        if(!isRightDriveSensorConnected()) {
            TelemetryUtil.print("right drive encoder is not connected", PrintStyle.ERROR);
            hasEmergency = true;
        }

        if(!isLeftDriveSensorConnected()) {
            TelemetryUtil.print("left drive encoder is not connected", PrintStyle.ERROR);
            hasEmergency = true;
        }   

        motionPlanner = new DriveMotionPlanner();

        DoubleSupplier drivePIDSupplier = () -> RobotState.visionTarget.getTargetArea();
        visionDrivePID = new PIDController(0, 0, 0, 0, drivePIDSupplier); //TODO: Add PID and range values
        visionDrivePID.setDesiredValue(DESIRED_TARGET_AREA);
        visionDrivePID.setMinMaxOutput(-0.7, 0.7);
        visionDrivePID.setIRange(0); //TODO: set appropriate I range
        visionDrivePID.disableDebug();
        visionDrivePID.reset();

        DoubleSupplier turnPIDSupplier = () -> RobotState.visionTarget.getXOffset();
        visionTurnPID = new PIDController(0, 0, 0, 0, turnPIDSupplier); //TODO: Add PID and range values
        visionTurnPID.setDesiredValue(0);
        visionTurnPID.setMinMaxOutput(-0.7, 0.7);
        visionTurnPID.setIRange(0); //TODO: set appropriate I range
        visionTurnPID.disableDebug();
        visionTurnPID.reset();



    }

    public boolean isRightDriveSensorConnected() {
        int pulseWidthPeriod = rightMaster.getSensorCollection().getPulseWidthRiseToRiseUs();
        return pulseWidthPeriod != 0;
    }

    public boolean isLeftDriveSensorConnected() {
        int pulseWidthPeriod = leftMaster.getSensorCollection().getPulseWidthRiseToRiseUs();
        return pulseWidthPeriod != 0;
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.driveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSec(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.driveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecToRpm(double inches_per_sec) {
        return inchesToRotations(inches_per_sec) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    public boolean isBrakeMode() {
        return isBrakeMode;
    }



    public synchronized void setBrakeMode(boolean on) {
        if(isBrakeMode != on) {
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            for(LazyTalonSRX motor : motors) {
                motor.setNeutralMode(mode);
            }
            isBrakeMode = on;
        }
    }


    public synchronized void setOpenLoop(DriveSignal signal) {
        if(state != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            leftMaster.configNeutralDeadband(0.04, 0);
            rightMaster.configNeutralDeadband(0.04, 0);
            Limelight.getInstance().ledsOn(false);
            state = DriveControlState.OPEN_LOOP;
        }

        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
        periodicIO.left_feedforward = 0.0;
        periodicIO.right_feedforward = 0.0;
    }


    public synchronized void setVisionControl(DriveSignal signal) {
        setVisionControl();
        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
        periodicIO.left_feedforward = 0.0;
        periodicIO.right_feedforward = 0.0;
    }

    public synchronized void setVisionControl() {
        if(state != DriveControlState.VISION) {
            setBrakeMode(true);
            leftMaster.configNeutralDeadband(0.0, 0);
            rightMaster.configNeutralDeadband(0.0, 0);
            Limelight.getInstance().ledsOn(true);
            visionDrivePID.reset();
            visionTurnPID.reset();
            state = DriveControlState.VISION;
        }
    }


    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedForward) {
        if(state != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            leftMaster.selectProfileSlot(0, 0);
            rightMaster.selectProfileSlot(0, 0);
            leftMaster.configNeutralDeadband(0);
            rightMaster.configNeutralDeadband(0);
            Limelight.getInstance().ledsOn(false);
            state = DriveControlState.PATH_FOLLOWING;
        }

        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
        periodicIO.left_feedforward = feedForward.getLeft();
        periodicIO.right_feedforward = feedForward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if(motionPlanner != null) {
            overrideTrajectory = false;
            motionPlanner.reset();
            motionPlanner.setTrajectory(trajectory);
            state = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if(motionPlanner == null || state != DriveControlState.PATH_FOLLOWING) {
            return true;
        }
        return motionPlanner.isDone() || overrideTrajectory;
    }


    public synchronized Rotation2d getHeading() {
        return periodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(Navx.getInstance().getHeading()));
        periodicIO.gyro_heading = gyroOffset;
    }
    
    public synchronized void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0, 0, 50);
        rightMaster.setSelectedSensorPosition(0, 0, 50);
        periodicIO = new PeriodicIO();
    }

    public double getLeftEncoderRotations() {
        return periodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return periodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getLeftVelocityNativeUnits() {
        return periodicIO.left_velocity_ticks_per_100ms;
    }

    public double getRightVelocityNativeUnits() {
        return periodicIO.right_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10 / DRIVE_ENCODER_PPR);
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public void overrideTrajectory(boolean override) {
        overrideTrajectory = override;
    }

    private void updatePathFollower() {
        if(state == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = motionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            periodicIO.error = motionPlanner.error();
            periodicIO.path_setpoint = motionPlanner.setpoint();

            if(!overrideTrajectory) {
                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), 
                    radiansPerSecondToTicksPer100ms(output.right_velocity)), new DriveSignal(output.left_feedforward_voltage/12.0,
                    output.right_feedforward_voltage/12.0));
                
                periodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                periodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                periodicIO.left_accel = periodicIO.right_accel = 0;
            }
        } else {
            TelemetryUtil.print("Drive is not in a path following state", PrintStyle.ERROR);
        }
    }

    private void updateVisionTargetFollower() {
        if(state == DriveControlState.VISION) {
            if(RobotState.visionTarget.isTargetVisible()) {
                double driveSpeed = visionDrivePID.getOutput();
                double turnSpeed = visionTurnPID.getOutput();
                setVisionControl(new DriveSignal(driveSpeed + turnSpeed, driveSpeed - turnSpeed));
            } else {
                setVisionControl(DriveSignal.BRAKE);
            }
            
        } else {
            TelemetryUtil.print("Drive is not in a vision target following state", PrintStyle.ERROR);
        }

    }

    public boolean isDoneWithTargetFollowing() {
        if(state != DriveControlState.VISION || !RobotState.visionTarget.isTargetVisible()) {
            return true;
        }
        return RobotState.visionTarget.hasReachedTarget();
    }



    private final Loop loop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            synchronized(Drivetrain.this) {
                setOpenLoop(new DriveSignal(0.0, 0.0));
                setBrakeMode(false);
            }
        }
    
        @Override
        public void onLoop(double timestamp) {
            synchronized(Drivetrain.this) {
                switch(state) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    case VISION:
                        updateVisionTargetFollower();
                        break;
                    default:
                        TelemetryUtil.print("unexpected drive control state", PrintStyle.ERROR);
                        break;
                }
            }
            
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = periodicIO.left_position_ticks;
        double prevRightTicks = periodicIO.right_position_ticks;
        periodicIO.left_position_ticks = leftMaster.getSelectedSensorPosition(0);
        periodicIO.right_position_ticks = rightMaster.getSelectedSensorPosition(0);
        periodicIO.left_velocity_ticks_per_100ms = leftMaster.getSelectedSensorVelocity(0);
        periodicIO.right_velocity_ticks_per_100ms = rightMaster.getSelectedSensorVelocity(0);
        periodicIO.gyro_heading = Rotation2d.fromDegrees(Navx.getInstance().getHeading()).rotateBy(gyroOffset);

        double deltaLeftTicks = ((periodicIO.left_position_ticks - prevLeftTicks) / DRIVE_ENCODER_PPR) * Math.PI;
        periodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;

        double deltaRightTicks = ((periodicIO.right_position_ticks -  prevRightTicks) / DRIVE_ENCODER_PPR) * Math.PI;
        periodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if(state == DriveControlState.OPEN_LOOP || state == DriveControlState.VISION) {
            leftMaster.set(ControlMode.PercentOutput, periodicIO.left_demand, DemandType.ArbitraryFeedForward, 0);
            rightMaster.set(ControlMode.PercentOutput, periodicIO.right_demand, DemandType.ArbitraryFeedForward, 0);
        } else {
            //TODO: Make sure this is correct
            leftMaster.set(ControlMode.Velocity, periodicIO.left_demand, DemandType.ArbitraryFeedForward, 
                periodicIO.left_feedforward + Constants.kDriveVelocityKd * periodicIO.left_accel / 1023.0);
            rightMaster.set(ControlMode.Velocity, periodicIO.right_demand, DemandType.ArbitraryFeedForward,
                periodicIO.right_feedforward + Constants.kDriveVelocityKd * periodicIO.left_accel / 1023.0);
        }
    }


    @Override
    public void zeroSensors() {
        resetEncoders();
    }
    
    @Override
    public void outputTelemetry() {
        if(Constants.showDebugOutput) {
            SmartDashboard.putNumber("Right Drive Distance", periodicIO.right_distance);
            SmartDashboard.putNumber("Right Drive Ticks", periodicIO.right_position_ticks);
            SmartDashboard.putNumber("Left Drive Distance", periodicIO.left_distance);
            SmartDashboard.putNumber("Left Drive Ticks", periodicIO.left_position_ticks);
            SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
            SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

            SmartDashboard.putNumber("x_err", periodicIO.error.getTranslation().x());
            SmartDashboard.putNumber("y_err", periodicIO.error.getTranslation().y());
            SmartDashboard.putNumber("theta_err", periodicIO.error.getRotation().getDegrees());
            if(getHeading() != null) {
                SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
            }
        }
    }


    @Override
    public void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }


    public Request openLoopRequest(DriveSignal signal) {
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(signal);
            }
        };
    }

    public Request alignToTargetRequest() {
        return new Request(){
        
            @Override
            public void act() {
                setVisionControl();
            }

            @Override
            public boolean isFinished() {
                return isDoneWithTargetFollowing();
            }
        };
    }

    public Request visionRequest() {
        return new Request(){
        
            @Override
            public void act() {
                setVisionControl();
            }
        };
    }

    public Request trajectoryRequest(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        return new Request(){
        
            @Override
            public void act() {
                setTrajectory(trajectory);
            }

            @Override
            public boolean isFinished() {
                return isDoneWithTrajectory();
            }
        };
    }

    public Request startTrajectoryRequest(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        return new Request(){
        
            @Override
            public void act() {
                setTrajectory(trajectory);
            }
        };
    }
    

    public enum DriveControlState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        VISION;
    }

    public static class PeriodicIO {
        //Inputs
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        //Outputs
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

}
