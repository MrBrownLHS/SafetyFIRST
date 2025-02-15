// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CollectorArm extends SubsystemBase {
  private final SparkMax liftMotor, pivotMotor, reachMotor, topIntakeMotor, bottomIntakeMotor;
  private final CANcoder liftEncoder, pivotEncoder, reachEncoder;
  private final PIDController liftPIDController, pivotPIDController, reachPIDController;
  private final CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig, reachEncoderConfig;
  private final SparkMaxConfig liftMotorConfig, pivotMotorConfig, reachMotorConfig, topIntakeMotorConfig, bottomIntakeMotorConfig;
  private final MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig, reachMagnetSensorConfig;
  private final DigitalInput coralLimit, algaeLimit;
 
  public CollectorArm() { 
    // Declare motors
    liftMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
    liftMotorConfig = new SparkMaxConfig();
    liftEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_ID);
    configureLiftMotor();
    pivotMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotMotorConfig = new SparkMaxConfig();
    pivotEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_ID);
    configurePivotMotor();
    reachMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_MOTOR_ID, MotorType.kBrushless);
    reachMotorConfig = new SparkMaxConfig();
    reachEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_ID);
    configureReachMotor();
    topIntakeMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    topIntakeMotorConfig = new SparkMaxConfig();
    bottomIntakeMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);
    bottomIntakeMotorConfig = new SparkMaxConfig();
    coralLimit = new DigitalInput(Constants.CollectorArmConstants.COLLECTOR_ARM_CORAL_LIMIT_ID);
    algaeLimit = new DigitalInput(Constants.CollectorArmConstants.COLLECTOR_ARM_ALGAE_LIMIT_ID);

    liftPIDController = new PIDController(
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kP, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kI, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kD);
    liftPIDController.setTolerance(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_TOLERANCE);

    pivotPIDController = new PIDController(
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kP, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kI,
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kD);
    pivotPIDController.setTolerance(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_TOLERANCE);

    reachPIDController = new PIDController(
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kP, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kI,
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kD);
    reachPIDController.setTolerance(Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_TOLERANCE);
    
        // Set encoder conversion factor to return values in degrees
    liftEncoderConfig = new CANcoderConfiguration();
    liftMagnetSensorConfig = new MagnetSensorConfigs();
    liftMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.COLLECTOR_ARM_ENCODER_CONVERSION_FACTOR;
    liftMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    liftEncoderConfig.MagnetSensor = liftMagnetSensorConfig;
    liftEncoder.getConfigurator().apply(liftEncoderConfig);

    pivotEncoderConfig = new CANcoderConfiguration();
    pivotMagnetSensorConfig = new MagnetSensorConfigs();
    pivotMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.COLLECTOR_ARM_ENCODER_CONVERSION_FACTOR;
    pivotMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pivotEncoderConfig.MagnetSensor = pivotMagnetSensorConfig;
    pivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    reachEncoderConfig = new CANcoderConfiguration();
    reachMagnetSensorConfig = new MagnetSensorConfigs();
    reachMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.COLLECTOR_ARM_ENCODER_CONVERSION_FACTOR;
    reachMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    reachEncoderConfig.MagnetSensor = reachMagnetSensorConfig;
    reachEncoder.getConfigurator().apply(reachEncoderConfig);

        // Configure PID (Tune these values!)
    configurePID(liftPIDController);
    configurePID(pivotPIDController);
    configurePID(reachPIDController);
    }
    private void configureLiftMotor() {
      var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_CURRENT_LIMIT);
        config.secondaryCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_MAX_CURRENT_LIMIT);
        config.voltageCompensation(Constants.CollectorArmConstants.COLLECTOR_ARM_VOLTAGE_COMPENSATION);
        liftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configurePivotMotor() {
      var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_CURRENT_LIMIT);
        config.secondaryCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_MAX_CURRENT_LIMIT);
        config.voltageCompensation(Constants.CollectorArmConstants.COLLECTOR_ARM_VOLTAGE_COMPENSATION);
        pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureReachMotor() {
      var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_CURRENT_LIMIT);
        config.secondaryCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_MAX_CURRENT_LIMIT);
        config.voltageCompensation(Constants.CollectorArmConstants.COLLECTOR_ARM_VOLTAGE_COMPENSATION);
        reachMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configurePID(PIDController pidController) {
        pidController.setP(0.01); // Adjust as needed
        pidController.setI(0.0);
        pidController.setD(0.01);
        pidController.setIntegratorRange(-1.0, 1.0);
    }

    public void setPosition(double liftAngle, double pivotAngle, double reachAngle) {
        double liftOutput = liftPIDController.calculate(liftAngle);
        liftMotor.set(liftOutput);

        double pivotOutput = pivotPIDController.calculate(pivotAngle);
        pivotMotor.set(pivotOutput);

        double reachOutput = reachPIDController.calculate(reachAngle);
        reachMotor.set(reachOutput);
    }

    // Commands to move to preset positions
    public void moveToStart() {
        setPosition(
          Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_START, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_START, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_START);
    }

    public void moveToPickup() {
        setPosition(
          Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_PICKUP, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_PICKUP, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_PICKUP);
    }

    public void moveToL1() {
        setPosition(
          Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_L1, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_L1, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_L1);
    }

    public void moveToL2() {
        setPosition(
          Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_L2, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_L2, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_L2);
    }

    public void moveToL3() {
        setPosition(
          Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_L3, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_L3, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_L3);
    }

    public void moveToMax() {
        setPosition(
          Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_MAX, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_MAX, 
          Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_MAX);
    }

    public void CollectAlgae(){
      if (!algaeLimit.get()) { // If limit switch is not pressed, run motor
        bottomIntakeMotor.set(Constants.CollectorArmConstants.COLLECTOR_ARM_INTAKE_SPEED);
        topIntakeMotor.set(Constants.CollectorArmConstants.COLLECTOR_ARM_INTAKE_SPEED);
    } else { // If pressed, stop motor
        bottomIntakeMotor.stopMotor();
        topIntakeMotor.stopMotor();
    }
    }

    public void CollectCoral(){
      if (!coralLimit.get()) { // If limit switch is not pressed, run motor
        bottomIntakeMotor.set(Constants.CollectorArmConstants.COLLECTOR_ARM_INTAKE_SPEED);
    } else { // If pressed, stop motor
        bottomIntakeMotor.stopMotor();
    }
    }

    public void ReleasePiece(){
      topIntakeMotor.set(Constants.CollectorArmConstants.COLLECTOR_ARM_OUTTAKE_SPEED);
      bottomIntakeMotor.set(Constants.CollectorArmConstants.COLLECTOR_ARM_OUTTAKE_SPEED);
    }

    public void YeetPiece(){
      topIntakeMotor.set(Constants.CollectorArmConstants.COLLECTOR_ARM_INTAKE_SPEED);
      bottomIntakeMotor.set(Constants.CollectorArmConstants.COLLECTOR_ARM_OUTTAKE_SPEED);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
