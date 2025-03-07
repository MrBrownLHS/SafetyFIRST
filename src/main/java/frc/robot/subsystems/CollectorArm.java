// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//https://chatgpt.com/share/67b1689b-d7d8-800e-8877-33cd210398d7
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.CollectorArm.CollectorArmState;
import frc.robot.utilities.ArmConfiguration;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation; 

public class CollectorArm extends SubsystemBase {
  private final ArmConfiguration armConfig;
  private boolean tuningMode = false;
  private ArmFeedforward liftFeedforward, pivotFeedforward;
  private TrapezoidProfile.Constraints liftConstraints, pivotConstraints;
  private TrapezoidProfile liftProfile, pivotProfile;
  private TrapezoidProfile.State liftGoal, pivotGoal, liftState, pivotState;
  private SlewRateLimiter rateLimiter;
  private final PIDController liftPIDController, pivotPIDController;

  public enum CollectorArmState { //adjust angles as needed
    START(0, 0),
    FLOOR(5, -45),
    COLLECT(10, -30),
    L1(20, -15),
    L2(30, 0),
    L3(40, 15),
    MAX(50, 30);

    public final double liftHeightInches, pivotAngleDegrees;
    CollectorArmState(double liftHeight, double pivotAngle) {
        this.liftHeightInches = liftHeight;
        this.pivotAngleDegrees = pivotAngle;
      }
    
  }

  public CollectorArm() { 
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.log("CollectorArm Subsystem Initialized");
    armConfig = new ArmConfiguration();
    SmartDashboard.putBoolean("Tuning Mode", false);
       
    rateLimiter = new SlewRateLimiter(Constants.CollectorArmConstants.ARTICULATE_RATE_LIMIT);

  
}

  private CollectorArmState currentState = CollectorArmState.START; // Default state

  public void setLiftPosition(double targetInches) {
      TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Constants.CollectorArmConstants.LIFT_MAX_VELOCITY,
          Constants.CollectorArmConstants.LIFT_MAX_ACCELERATION
      );
      TrapezoidProfile.State liftState =
      new TrapezoidProfile.State(this.liftState.position, this.liftState.velocity);
      TrapezoidProfile liftProfile = new TrapezoidProfile(constraints);

      liftState = liftProfile.calculate(0.02, liftState, liftGoal);
      double pidOutput = liftPIDController.calculate(getLiftHeightInches(), liftState.position);
    
      double ffOutput = liftFeedforward.calculate(liftState.velocity, 0);
      armConfig.getliftMotor1.set(pidOutput + ffOutput);
      armConfig.getliftMotor2.set(-pidOutput + ffOutput);
}

  public void setPivotPosition(double targetAngle) {
      TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(
        Constants.CollectorArmConstants.PIVOT_MAX_VELOCITY,
        Constants.CollectorArmConstants.PIVOT_MAX_ACCELERATION
      );
      TrapezoidProfile.State pivotState =
        new TrapezoidProfile.State(targetAngle, 0);
      TrapezoidProfile pivotProfile = new TrapezoidProfile(constraints);
        
      pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);

      double pidOutput = pivotPIDController.calculate(getPivotAngleDegrees(), pivotState.position);
  
      double ffOutput = pivotFeedforward.calculate(pivotState.velocity, 0);
      pivotMotor1.set(pidOutput + ffOutput);
      pivotMotor2.set(-pidOutput + ffOutput);
}


    public void moveToState(CollectorArmState state) {
      setLiftPosition(state.liftHeightInches);
      setPivotPosition(state.pivotAngleDegrees);
}
    public boolean isAtTarget(CollectorArmState state) {
      return liftPIDController.atSetpoint() && pivotPIDController.atSetpoint();
    }

    public void stopArm() {
      liftMotor1.stopMotor();
      liftMotor2.stopMotor();
      pivotMotor1.stopMotor();
      pivotMotor2.stopMotor();
      topIntakeMotor.stopMotor();
      bottomIntakeMotor.stopMotor();
      articulateMotor.stopMotor();
      DataLogManager.log("Emergency Stop Triggered!");
  }

   
    public RunCommand ArticulateCollector(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = rateLimiter.calculate(adjustedInput);
        articulateMotor.set(limitedInput);
      }, this);
    }
  
    public RunCommand CollectAlgae(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = rateLimiter.calculate(adjustedInput);
        bottomIntakeMotor.set(-limitedInput); //May need to switch which motor is inverted so they run opposite directions
        topIntakeMotor.set(limitedInput);
      }, this);
    }

    public RunCommand CollectCoral(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = rateLimiter.calculate(adjustedInput);
        bottomIntakeMotor.set(limitedInput); 
      }, this);
    }

    public RunCommand YeetAlgae(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        topIntakeMotor.set(Constants.CollectorArmConstants.YEET_SPEED);
        bottomIntakeMotor.set(Constants.CollectorArmConstants.YEET_SPEED);
      }, this);
    }

    public RunCommand AutoCoral() {
      return new RunCommand(() -> {
        bottomIntakeMotor.set(-0.5);
      }, this);
    }

  @Override
  public void periodic() {
    updateDashboard();
    logArmState();
    // This method will be called once per scheduler run
  }
}
