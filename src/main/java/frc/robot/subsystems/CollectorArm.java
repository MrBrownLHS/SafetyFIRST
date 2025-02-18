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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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
  private final SparkMax liftMotor, pivotMotor, topIntakeMotor, bottomIntakeMotor;
  private final CANcoder liftEncoder, pivotEncoder;
  private final PIDController liftPIDController, pivotPIDController;
  private final CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig;
  private final SparkMaxConfig liftMotorConfig, pivotMotorConfig, topIntakeMotorConfig, bottomIntakeMotorConfig;
  private final MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig;
  private final DigitalInput coralLimit, algaeLimit;
  private final ArmFeedforward liftFeedforward, pivotFeedforward;
  private TrapezoidProfile.Constraints liftConstraints, pivotConstraints;
  private TrapezoidProfile.State liftState, pivotState;
  private TrapezoidProfile liftProfile, pivotProfile;
  private final TrapezoidProfile.State liftGoal, pivotGoal;
  

  public enum CollectorArmState { //adjust angles as needed
    START(0, 0),
    FLOOR(5, -45),
    COLLECT(10, -30),
    L1(20, -15),
    L2(30, 0),
    L3(40, 15),
    MAX(50, 30);

    public final double liftHeightInches;
    public final double pivotAngleDegrees;

    CollectorArmState(double liftHeight, double pivotAngle) {
        this.liftHeightInches = liftHeight;
        this.pivotAngleDegrees = pivotAngle;
    }
}




 
  public CollectorArm() { 
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    
    DataLogManager.log("CollectorArm Subsystem Initialized");
    
    liftMotor = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_ID, MotorType.kBrushless);
    liftMotorConfig = new SparkMaxConfig();
    liftEncoder = new CANcoder(Constants.CollectorArmConstants.LIFT_ENCODER_ID);
    pivotMotor = new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotMotorConfig = new SparkMaxConfig();
    pivotEncoder = new CANcoder(Constants.CollectorArmConstants.PIVOT_ENCODER_ID);
    topIntakeMotor = new SparkMax(Constants.CollectorArmConstants.TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    topIntakeMotorConfig = new SparkMaxConfig();
    bottomIntakeMotor = new SparkMax(Constants.CollectorArmConstants.BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);
    bottomIntakeMotorConfig = new SparkMaxConfig();
    coralLimit = new DigitalInput(Constants.CollectorArmConstants.CORAL_LIMIT_ID);
    algaeLimit = new DigitalInput(Constants.CollectorArmConstants.ALGAE_LIMIT_ID);

    liftConstraints = new TrapezoidProfile.Constraints(
      Constants.CollectorArmConstants.LIFT_MAX_VELOCITY, 
      Constants.CollectorArmConstants.LIFT_MAX_ACCELERATION);
    pivotConstraints = new TrapezoidProfile.Constraints(
      Constants.CollectorArmConstants.PIVOT_MAX_VELOCITY, 
      Constants.CollectorArmConstants.PIVOT_MAX_ACCELERATION);

    liftState = new TrapezoidProfile.State(0, 0);
    pivotState = new TrapezoidProfile.State(0, 0);

    liftGoal = new TrapezoidProfile.State(0, 0);
    pivotGoal = new TrapezoidProfile.State(0, 0);

    TrapezoidProfile.State liftGoal = new TrapezoidProfile.State(0, 0);
      liftState = liftProfile.calculate(0.02, liftState, liftGoal);
    TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State(0, 0);
      pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);

    liftProfile = new TrapezoidProfile(liftConstraints);
    pivotProfile = new TrapezoidProfile(pivotConstraints);
            
    liftFeedforward = new ArmFeedforward(
      Constants.CollectorArmConstants.LIFT_kS, 
      Constants.CollectorArmConstants.LIFT_kG, 
      Constants.CollectorArmConstants.LIFT_kV, 
      Constants.CollectorArmConstants.LIFT_kA);
    pivotFeedforward = new ArmFeedforward(
      Constants.CollectorArmConstants.PIVOT_kS, 
      Constants.CollectorArmConstants.PIVOT_kG, 
      Constants.CollectorArmConstants.PIVOT_kV, 
      Constants.CollectorArmConstants.PIVOT_kA);
    
    liftPIDController = new PIDController(
      Constants.CollectorArmConstants.LIFT_kP, 
      Constants.CollectorArmConstants.LIFT_kI, 
      Constants.CollectorArmConstants.LIFT_kD);
    liftPIDController.setTolerance(Constants.CollectorArmConstants.LIFT_TOLERANCE);

    pivotPIDController = new PIDController(
      Constants.CollectorArmConstants.PIVOT_kP, 
      Constants.CollectorArmConstants.PIVOT_kI,
      Constants.CollectorArmConstants.PIVOT_kD);
    pivotPIDController.setTolerance(Constants.CollectorArmConstants.PIVOT_TOLERANCE);

        
        // Set encoder conversion factor to return values in degrees
    liftEncoderConfig = new CANcoderConfiguration();
    liftMagnetSensorConfig = new MagnetSensorConfigs();
    liftMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.ENCODER_TO_INCHES;
    liftMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    liftEncoderConfig.MagnetSensor = liftMagnetSensorConfig;
    liftEncoder.getConfigurator().apply(liftEncoderConfig);

    pivotEncoderConfig = new CANcoderConfiguration();
    pivotMagnetSensorConfig = new MagnetSensorConfigs();
    pivotMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.ENCODER_TO_DEGREES;
    pivotMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pivotEncoderConfig.MagnetSensor = pivotMagnetSensorConfig;
    pivotEncoder.getConfigurator().apply(pivotEncoderConfig);

            // Configure PID (Tune these values!)
    configurePID(liftPIDController, Constants.CollectorArmConstants.LIFT_kP,
      Constants.CollectorArmConstants.LIFT_kI, Constants.CollectorArmConstants.LIFT_kD);

    configurePID(pivotPIDController, Constants.CollectorArmConstants.PIVOT_kP,
      Constants.CollectorArmConstants.PIVOT_kI, Constants.CollectorArmConstants.PIVOT_kD);
    
    configureMotor(liftMotor, liftMotorConfig);
    configureMotor(pivotMotor, pivotMotorConfig);
    configureMotor(topIntakeMotor, topIntakeMotorConfig);
    configureMotor(bottomIntakeMotor, bottomIntakeMotorConfig);
    
    }

  public void updateDashboard() {
    SmartDashboard.putNumber("Lift Height", liftEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Lift Setpoint", liftPIDController.getSetpoint());
    SmartDashboard.putNumber("Pivot Setpoint", pivotPIDController.getSetpoint());
    SmartDashboard.putNumber("Lift PID Error", liftPIDController.getPositionError());
    SmartDashboard.putNumber("Pivot PID Error", pivotPIDController.getPositionError());
    SmartDashboard.putNumber("Lift Motor Output", liftMotor.get());
    SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.get());
    SmartDashboard.putBoolean("Algae Limit Switch", algaeLimit.get());
    SmartDashboard.putBoolean("Coral Limit Switch", coralLimit.get());
    SmartDashboard.putString("Current Arm State", currentState.name());
  }

  public void logArmState() {
    DataLogManager.log("Current State: " + currentState.name());
    DataLogManager.log("Lift Height: " + liftEncoder.getPosition().getValueAsDouble());
    DataLogManager.log("Pivot Angle: " + pivotEncoder.getPosition().getValueAsDouble());
    DataLogManager.log("Lift Output: " + liftMotor.get());
    DataLogManager.log("Pivot Output: " + pivotMotor.get());
}

  private void configureMotor(SparkMax motor, SparkMaxConfig config) {
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.CollectorArmConstants.CURRENT_LIMIT);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT);
    config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

  private void configurePID(PIDController pidController, double kP, double kI, double kD) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
  }

  public double getLiftHeightInches() {
    return liftEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_INCHES;
}

  public double getPivotAngleDegrees() {
    return pivotEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_DEGREES;
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
      liftMotor.set(pidOutput + ffOutput);
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
      pivotMotor.set(pidOutput + ffOutput);
}


    public void moveToState(CollectorArmState state) {
      setLiftPosition(state.liftHeightInches);
      setPivotPosition(state.pivotAngleDegrees);
}
    public boolean isAtTarget(CollectorArmState state) {
      return liftPIDController.atSetpoint() && pivotPIDController.atSetpoint();
    }

    public void stopArm() {
      liftMotor.stopMotor();
      pivotMotor.stopMotor();
      topIntakeMotor.stopMotor();
      bottomIntakeMotor.stopMotor();
      DataLogManager.log("Emergency Stop Triggered!");
  }

    private boolean algaeCollected = false;
    private boolean coralCollected = false;

    public void CollectAlgae(){
      if (!algaeLimit.get()) { 
        algaeCollected = false;
        bottomIntakeMotor.set(Constants.CollectorArmConstants.INTAKE_SPEED);
        topIntakeMotor.set(Constants.CollectorArmConstants.INTAKE_SPEED);
    } else if (!algaeCollected) {
        bottomIntakeMotor.stopMotor();
        topIntakeMotor.stopMotor();
        algaeCollected = true;
    }
    }

    public void CollectCoral(){
      if (!coralLimit.get()) {
          coralCollected = false;
          bottomIntakeMotor.set(Constants.CollectorArmConstants.INTAKE_SPEED);
      } else if (!coralCollected) {
          bottomIntakeMotor.stopMotor();
          coralCollected = true;
      }
  }

    public void ReleasePiece(){
      topIntakeMotor.set(Constants.CollectorArmConstants.OUTTAKE_SPEED);
      bottomIntakeMotor.set(Constants.CollectorArmConstants.OUTTAKE_SPEED);
    }

    public void YeetPiece(){
      topIntakeMotor.set(Constants.CollectorArmConstants.YEET_SPEED);
      bottomIntakeMotor.set(Constants.CollectorArmConstants.YEET_SPEED);
    }

  @Override
  public void periodic() {
    updateDashboard();
    logArmState();
    // This method will be called once per scheduler run
  }
}
