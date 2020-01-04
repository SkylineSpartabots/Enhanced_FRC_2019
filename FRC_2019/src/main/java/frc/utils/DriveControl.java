/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utils;

import frc.robot.states.DriveSignal;

/**
 * Add your docs here.
 */
public class DriveControl {
    private static final double turnNonLinearity = 0.6;
    
    private static final double negInertiaThreshold = 0.65;
    private static final double negInertiaTurnScalar = 3.5;
    private static final double negInertiaCloseScalar = 4.0;
    private static final double negInertiaFarScalar = 5.0;

    private static final double sensitivity = 0.65;

    private static final double quickStopDeadband = 0.5;
    private static final double quickStopWeight = 0.1;
    private static final double quickStopScalar = 5.0;

    private double oldTurn = 0;
    private double quickStopAccumulator = 0;
    private double negInertiaAccumulator = 0;

    //TODO: relationship between motor speed 
    public DriveSignal curvatureDrive(double throttle, double turn, boolean isQuickTurn) {
        double negInertia = turn - oldTurn;
        oldTurn = turn; 
        final double denominator = Math.sin(Math.PI / 2.0 * turnNonLinearity);
        turn = Math.sin(Math.PI / 2.0 * turnNonLinearity * turn) / denominator;
        turn = Math.sin(Math.PI / 2.0 * turnNonLinearity * turn) / denominator;

        double leftPower, rightPower, overPower;
        double angularPower, linearPower;

        double negInertiaScalar;

        if(turn * negInertia > 0) {
            negInertiaScalar = negInertiaTurnScalar;
        } else {
            if(Math.abs(turn) > negInertiaThreshold) {
                negInertiaScalar = negInertiaCloseScalar;
            } else {
                negInertiaScalar = negInertiaFarScalar;
            }
        }

        double negInertiaPower = negInertia * negInertiaScalar;
        negInertiaAccumulator += negInertiaPower;

        turn = turn + negInertiaAccumulator;
        if(negInertiaAccumulator > 1) {
            negInertiaAccumulator -= 1;
        } else if (negInertiaAccumulator < -1) {
            negInertiaAccumulator += -1;
        } else {
            negInertiaAccumulator = 0;
        }

        linearPower = throttle;

        if(isQuickTurn) {
            if(Math.abs(linearPower) < quickStopDeadband) {
                double alpha = quickStopWeight;
                quickStopAccumulator = (1-alpha) * quickStopAccumulator
                    + alpha * Util.limit(turn, 1.0) * quickStopScalar;
            }
            overPower = 1.0;
            angularPower = turn;
        } else {
            overPower = 0;
            angularPower = Math.abs(throttle) * turn * sensitivity - quickStopAccumulator;
            if(quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }

        rightPower = leftPower = linearPower;
        rightPower += angularPower;
        leftPower -= angularPower;

        if(leftPower > 1) {
            rightPower -= overPower * (leftPower - 1);
            leftPower = 1;
        } else if (rightPower > 1) {
            leftPower -= overPower * (rightPower - 1);
            rightPower = 1;
        } else if (leftPower < -1) {
            rightPower += overPower * (-1.0 - leftPower);
            leftPower = -1;
        } else if(rightPower < -1) {
            leftPower += overPower * (-1.0 - rightPower);
            rightPower = -1;
        }

        return new DriveSignal(leftPower, rightPower);

    }


    public DriveSignal arcadeDrive(double throttle, double turn) {
        /*throttle = Util.clipToOutput(throttle);
      
        turn = Util.clipToOutput(turn);
      
        throttle = Math.copySign(throttle * throttle, throttle);
        turn = Math.copySign(turn * turn, turn);
      
        double leftMotorOutput;
        double rightMotorOutput;
      
        double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(turn)), throttle);
      
        if (throttle >= 0.0) {
            // First quadrant, else second quadrant
            if (turn >= 0.0) {
              leftMotorOutput = maxInput;
              rightMotorOutput = throttle - turn;
            } else {
              leftMotorOutput = throttle + turn;
              rightMotorOutput = maxInput;
            }
          } else {
            // Third quadrant, else fourth quadrant
            if (turn >= 0.0) {
              leftMotorOutput = throttle + turn;
              rightMotorOutput = maxInput;
            } else {
              leftMotorOutput = maxInput;
              rightMotorOutput = throttle - turn;
            }
          }
      
          return new DriveSignal(leftMotorOutput, rightMotorOutput);
    }*/

    return new DriveSignal(throttle + turn, throttle - turn);
    }

}
