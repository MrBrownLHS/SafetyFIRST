// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

private final TrapezoidProfile.Constraints constraints = 
    new TrapezoidProfile.Constraints(2000, 1500);  // Max velocity & accel

private TrapezoidProfile.State lastState = new TrapezoidProfile.State(0, 0);

public void setSmoothPosition(double targetAngle) {
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    TrapezoidProfile.State goal = new TrapezoidProfile.State(targetAngle, 0);
    TrapezoidProfile.State setpoint = profile.calculate(0.02, lastState, goal);

    double pidOutput = liftPIDController.calculate(setpoint.position);
    liftMotor.set(pidOutput);

    lastState = setpoint;
}

public enum ArmState {
    START, PICKUP, L1, L2, L3, MAX
}

private ArmState currentState = ArmState.START;

public void moveToState(ArmState state) {
    currentState = state;
}

public void updateArm() {
    switch (currentState) {
        case START:
            setPosition(0, 0, 0);
            break;
        case PICKUP:
            setPosition(30, 15, 10);
            break;
        case L1:
            setPosition(45, 30, 20);
            break;
        case L2:
            setPosition(60, 40, 30);
            break;
        case L3:
            setPosition(75, 50, 40);
            break;
        case MAX:
            setPosition(90, 60, 50);
            break;
    }
}

private double liftOffset;

public void resetLiftEncoder() {
    liftOffset = liftEncoder.getAbsolutePosition().getValueAsDouble();
}

public double getLiftRelativePosition() {
    return liftEncoder.getAbsolutePosition().getValueAsDouble() - liftOffset;
}

private static final double DEADBAND = 0.05;  // Ignore small movements
private static final double MAX_SPEED = 0.8;  // Limit motor output

private void safeSetMotor(SparkMax motor, double output) {
    if (Math.abs(output) < DEADBAND) output = 0;
    motor.set(Math.max(-MAX_SPEED, Math.min(output, MAX_SPEED)));
}

safeSetMotor(liftMotor, liftOutput);
safeSetMotor(pivotMotor, pivotOutput);
safeSetMotor(reachMotor, reachOutput);

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public void updateDashboard() {
    SmartDashboard.putNumber("Lift Angle", liftEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Reach Angle", reachEncoder.getAbsolutePosition().getValueAsDouble());
}


/** Add your docs here. */
public class Reference {}
