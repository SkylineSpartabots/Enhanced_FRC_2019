/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.SmartDashboardInteractions.DriveStation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.TargetInfo;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Twist2d;
import frc.team254.lib.physics.Kinematics;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.util.InterpolatingDouble;
import frc.team254.lib.util.InterpolatingTreeMap;


/**
 * Add your docs here.
 */
public class RobotState {
    private static RobotState instance = null;
    public static RobotState getInstance() {
        if(instance == null) 
            instance = new RobotState();
        return instance;
    }

    private boolean seesTarget = false;
    public boolean seesTarget() {
        return seesTarget;
    }

    private static final int observationBufferSize = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> fieldToVehicle;
    private Twist2d predictedVelocity;
    private Twist2d measuredVelocity;
    private double distanceDriven;
    
    public static TargetInfo visionTarget = new TargetInfo();

    private DriveStation side;

    private RobotState() {
        reset(0, new Pose2d());
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        fieldToVehicle = new InterpolatingTreeMap<>(observationBufferSize);
        fieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        Drivetrain.getInstance().setHeading(initial_field_to_vehicle.getRotation());
        predictedVelocity = Twist2d.identity();
        measuredVelocity = Twist2d.identity();
        distanceDriven = 0;
        visionTarget = new TargetInfo();
    }

    public void setAutoSide(DriveStation side) {
        this.side = side;
    }

    public DriveStation getSide() {
        return side;
    }
    
    public synchronized void resetDistanceDrive() {
        distanceDriven = 0;
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return fieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return fieldToVehicle.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
            .transformBy(Pose2d.exp(predictedVelocity.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        fieldToVehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d measured_velocity, Twist2d predicted_velocity) {
       addFieldToVehicleObservation(timestamp, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), 
            measured_velocity));
        measuredVelocity = measured_velocity;
        predictedVelocity = predicted_velocity;
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta, double right_encoder_delta,
        Rotation2d current_gyro_heading) {
        final Pose2d lastMeasurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics(lastMeasurement.getRotation(), left_encoder_delta,
                right_encoder_delta, current_gyro_heading);
        distanceDriven += delta.dx;
        return delta;
    } 

    public synchronized double getDistanceDriven() {
        return distanceDriven;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return predictedVelocity;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return measuredVelocity;
    }

    public void outputToSmartDashboard() {
        Pose2d odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Robot Pose X", odometry.getTranslation().x());
        SmartDashboard.putNumber("Robot Pose Y", odometry.getTranslation().y());
        SmartDashboard.putNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Linear Velocity", measuredVelocity.dx);
        SmartDashboard.putBoolean("Is Target Visible", visionTarget.isTargetVisible());
        SmartDashboard.putNumber("Target Area", visionTarget.getTargetArea());
        SmartDashboard.putNumber("Target Angular Displacement", visionTarget.getXOffset());
    }



}
