// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//https://chatgpt.com/share/67b1689b-d7d8-800e-8877-33cd210398d7
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import frc.robot.utilities.ArmConfiguration;
import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.math.MathUtil;

public class CollectorArm extends SubsystemBase {
  private final ArmConfiguration armConfig;
  private boolean tuningMode = false;

  public enum CollectorArmState { //adjust angles as needed
    START(0, 0),
    COLLECT(10, -30),
    L1(20, -15),
    L2(30, 0),
    L3(40, 15);
    

  public final double liftHeightInches, pivotAngleDegrees;
    CollectorArmState(double liftHeight, double pivotAngle) {
        this.liftHeightInches = liftHeight;
        this.pivotAngleDegrees = pivotAngle;
      }
    
  }
  private final TrapezoidProfile liftProfile, pivotProfile;
  private CollectorArmState currentState;
  private TrapezoidProfile.State liftState, pivotState, liftGoal, pivotGoal;

  public CollectorArm() { 
    armConfig = new ArmConfiguration();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("CollectorArm Subsystem Initialized");
    SmartDashboard.putBoolean("Tuning Mode", false);

    liftProfile = new TrapezoidProfile(armConfig.liftConstraints);
    pivotProfile = new TrapezoidProfile(armConfig.pivotConstraints);
  }

  public void moveToState(CollectorArmState state) {
    currentState = state;

    double safeLiftHeight = MathUtil.clamp(state.liftHeightInches, 
      Constants.CollectorArmConstants.LIFT_MIN_HEIGHT, 
      Constants.CollectorArmConstants.LIFT_MAX_HEIGHT);

    double safePivotAngle = MathUtil.clamp(state.pivotAngleDegrees, 
      Constants.CollectorArmConstants.PIVOT_MIN_ANGLE, 
      Constants.CollectorArmConstants.PIVOT_MAX_ANGLE);

    liftGoal = new TrapezoidProfile.State(safeLiftHeight, 0);
    pivotGoal = new TrapezoidProfile.State(safePivotAngle, 0);

    double elapsedTime = 0.02;

    liftState = liftProfile.calculate(elapsedTime, liftState, liftGoal);
    pivotState = pivotProfile.calculate(elapsedTime, pivotState, pivotGoal);

    double liftFF = armConfig.liftFeedforward.calculate(liftState.velocity, 0);
    double pivotFF = armConfig.pivotFeedforward.calculate(pivotState.velocity, 0);

    double liftPID = armConfig.liftPIDController.calculate(getLiftHeightInches(), liftState.position);
    double pivotPID = armConfig.pivotPIDController.calculate(getPivotAngleDegrees(), pivotState.position);

    double smoothedLiftOutput = armConfig.rateLimiter.calculate(liftPID + liftFF);
    double smoothedPivotOutput = armConfig.rateLimiter.calculate(pivotPID + pivotFF);

    if (!isAtTarget(state))

    armConfig.m_Lift.set(smoothedLiftOutput);
    armConfig.m_Pivot.set(smoothedPivotOutput);
    
  } 
   
  public double getLiftHeightInches() {
    return armConfig.liftEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_INCHES;
  }

  public double getPivotAngleDegrees() {
    return armConfig.pivotEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_DEGREES;
  }
  
  public void setLiftPosition(double targetInches) {
    if (Math.abs(targetInches - getLiftHeightInches()) > Constants.CollectorArmConstants.LIFT_TOLERANCE) {
      liftGoal = new TrapezoidProfile.State(targetInches, 0);
      liftState = liftProfile.calculate(0.02, liftState, liftGoal);

      double pidOutput = armConfig.liftPIDController.calculate(getLiftHeightInches(), liftState.position);
      double ffOutput = armConfig.liftFeedforward.calculate(liftState.velocity, 0);

      armConfig.m_Lift.set(pidOutput + ffOutput);
      //armConfig.liftMotor2.set(pidOutput + ffOutput);
    }
  }

  public void setPivotPosition(double targetAngle) {
    if (Math.abs(targetAngle - getPivotAngleDegrees()) > Constants.CollectorArmConstants.PIVOT_TOLERANCE) {
      pivotGoal = new TrapezoidProfile.State(targetAngle, 0);
      pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);

      double pidOutput = armConfig.pivotPIDController.calculate(getLiftHeightInches(), pivotState.position);
      double ffOutput = armConfig.pivotFeedforward.calculate(pivotState.velocity, 0);

      armConfig.m_Pivot.set(pidOutput + ffOutput);
      //armConfig.pivotMotor2.set(pidOutput + ffOutput);
    }
  }

  public void stopArm() {
    SparkMax[] motors = {
      armConfig.m_Lift, armConfig.m_Pivot,
      armConfig.m_CoralIntake, armConfig.m_CoralArticulate,
      
    };

    for (SparkMax motor : motors) {
        motor.stopMotor();
    }

    DataLogManager.log("Emergency Stop Triggered!");
  }
 
    public boolean isAtTarget(CollectorArmState state) {
      return armConfig.liftPIDController.atSetpoint() && armConfig.pivotPIDController.atSetpoint();
    }

   
    public RunCommand ArticulateCoralCollector(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = armConfig.rateLimiter.calculate(adjustedInput);
        armConfig.m_CoralArticulate.set(limitedInput);
      }, this);
    }
  
    

    public RunCommand CollectCoral(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        armConfig.m_CoralIntake.set(rawInput);
      }, this);
    }

    public RunCommand AutoCollectCoral() {
      return new RunCommand(() -> {
        armConfig.m_CoralIntake.set(Constants.CollectorArmConstants.AUTO_CORAL_RELEASE_SPEED);
      }, this);
    }

 

  private void updateDashboard() {
    SmartDashboard.putNumber("Lift Height", getLiftHeightInches());
    SmartDashboard.putNumber("Pivot Angle", getPivotAngleDegrees());
    SmartDashboard.putString("Current Arm State", currentState.name());

    boolean newTuningMode = SmartDashboard.getBoolean("Tuning Mode", false);
    if (newTuningMode != tuningMode) {
      tuningMode = newTuningMode;
      armConfig.updatePIDTuning(tuningMode);
    }


  }

  @Override
  public void periodic() {
    updateDashboard();
    // This method will be called once per scheduler run
  }
}


