// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//https://chatgpt.com/share/67b1689b-d7d8-800e-8877-33cd210398d7
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;

import java.io.ObjectInputFilter.Config;
import edu.wpi.first.math.controller.ArmFeedforward;
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
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CollectorArm extends SubsystemBase {
  private final SparkMax liftMotor, pivotMotor, reachMotor, topIntakeMotor, bottomIntakeMotor;
  private final CANcoder liftEncoder, pivotEncoder, reachEncoder;
  private final PIDController liftPIDController, pivotPIDController, reachPIDController;
  private final CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig, reachEncoderConfig;
  private final SparkMaxConfig liftMotorConfig, pivotMotorConfig, reachMotorConfig, topIntakeMotorConfig, bottomIntakeMotorConfig;
  private final MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig, reachMagnetSensorConfig;
  private final DigitalInput coralLimit, algaeLimit;
  private final SmartMotionConfig configureSmartMotion;
  private final ArmFeedforward liftFeedforward, pivotFeedforward, reachFeedforward;+
  private final Timer timer = new Timer();
 
  public CollectorArm() { 
    // Declare motors
    timer.start();
    liftMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
    liftMotorConfig = new SparkMaxConfig();
    liftEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_ID);
    pivotMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotMotorConfig = new SparkMaxConfig();
    pivotEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_ID);
    reachMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_MOTOR_ID, MotorType.kBrushless);
    reachMotorConfig = new SparkMaxConfig();
    reachEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_ID);
    topIntakeMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    topIntakeMotorConfig = new SparkMaxConfig();
    bottomIntakeMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);
    bottomIntakeMotorConfig = new SparkMaxConfig();
    coralLimit = new DigitalInput(Constants.CollectorArmConstants.COLLECTOR_ARM_CORAL_LIMIT_ID);
    algaeLimit = new DigitalInput(Constants.CollectorArmConstants.COLLECTOR_ARM_ALGAE_LIMIT_ID);
    configureSmartMotion = new SmartMotionConfig();
    
    liftFeedforward = new ArmFeedforward(
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kS, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kG, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kV, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kA);
    pivotFeedforward = new ArmFeedforward(
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kS, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kG, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kV, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kA);

    reachFeedforward = new ArmFeedforward(
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kS, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kG, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kV, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kA);
    
    liftPIDController = new PIDController(
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kP, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kI, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kD,
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kF);
    liftPIDController.setTolerance(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_TOLERANCE);

    pivotPIDController = new PIDController(
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kP, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kI,
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kD,
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kF);
    pivotPIDController.setTolerance(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_TOLERANCE);

    reachPIDController = new PIDController(
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kP, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kI,
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kD,
      Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_kF);
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

    configureMotor(liftMotor, liftMotorConfig);
    configureMotor(pivotMotor, pivotMotorConfig);
    configureMotor(reachMotor, reachMotorConfig);
    configureMotor(topIntakeMotor, topIntakeMotorConfig);
    configureMotor(bottomIntakeMotor, bottomIntakeMotorConfig);

    configureSmartMotion(liftMotor, liftMotorConfig);
    configureSmartMotion(pivotMotor, pivotMotorConfig);
    configureSmartMotion(reachMotor, reachMotorConfig);
    }

    private void configureMotor(SparkMax motor, SparkMaxConfig config) {
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_CURRENT_LIMIT);
      config.secondaryCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_MAX_CURRENT_LIMIT);
      config.voltageCompensation(Constants.CollectorArmConstants.COLLECTOR_ARM_VOLTAGE_COMPENSATION);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      //var config = new SparkMaxConfig();
        //config.idleMode(IdleMode.kBrake);
        //config.smartCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_CURRENT_LIMIT);
        //config.secondaryCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_MAX_CURRENT_LIMIT);
        //config.voltageCompensation(Constants.CollectorArmConstants.COLLECTOR_ARM_VOLTAGE_COMPENSATION);
        //liftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }


    private void configurePID(PIDController pidController) {
      liftPIDController.setP(0.02);
      liftPIDController.setI(0.0);
      liftPIDController.setD(0.01);
      liftPIDController.setFF(0.00015);
      liftPIDController.setSmartMotionMaxVelocity(3000, 0);
      liftPIDController.setSmartMotionMaxAccel(2000, 0);
      liftPIDController.setSmartMotionAllowedClosedLoopError(0.1, 0);
    }


    private void configureSmartMotion(SparkMax motor, SparkMaxConfig config) {
      config.setMaxVelocity(3000);  // Max speed (adjust per need)
      config.smartMotionMinOutputVelocity(0.0);
      config.smartMotionMaxAccel(2000);     // Acceleration limit
      config.smartMotionAllowedClosedLoopError(0.1);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setLiftPosition (double targetAngle) {
      double currentAngle = liftEncoder.getAbsolutePosition().getValueAsDouble();
      double error = getShortestPath(currentAngle, targetAngle);
      double pidOutput = liftPIDController.calculate(currentAngle, targetAngle);
      double ffOutput = liftFeedforward.calculate(Math.toRadians(targetAngle),0);
      liftMotor.set(pidOutput + ffOutput);
    }

    private void setPivotPosition (double targetAngle) {
      double currentAngle = liftEncoder.getAbsolutePosition().getValueAsDouble();
      double error = getShortestPath(currentAngle, targetAngle);
      double output = liftPIDController.calculate(currentAngle, targetAngle);
      liftMotor.set(output * Math.signum(error));
    }

    private void setReachPosition (double targetAngle) {
      double currentAngle = liftEncoder.getAbsolutePosition().getValueAsDouble();
      double error = getShortestPath(currentAngle, targetAngle);
      double output = liftPIDController.calculate(currentAngle, targetAngle);
      liftMotor.set(output * Math.signum(error));
    }

    
  
  

    public void setPosition(double targetLiftAngle, double targetPivotAngle, double targetReachAngle) {
      double currentLiftAngle = liftEncoder.getAbsolutePosition().getValueAsDouble();
      double currentPivotAngle = pivotEncoder.getAbsolutePosition().getValueAsDouble();
      double currentReachAngle = reachEncoder.getAbsolutePosition().getValueAsDouble();
  
      double liftError = getShortestPath(currentLiftAngle, targetLiftAngle);
      double pivotError = getShortestPath(currentPivotAngle, targetPivotAngle);
      double reachError = getShortestPath(currentReachAngle, targetReachAngle);
  
      double liftOutput = liftPIDController.calculate(currentLiftAngle, targetLiftAngle);
      double pivotOutput = pivotPIDController.calculate(currentPivotAngle, targetPivotAngle);
      double reachOutput = reachPIDController.calculate(currentReachAngle, targetReachAngle);
  
      liftMotor.set(liftOutput * Math.signum(liftError)); 
      pivotMotor.set(pivotOutput * Math.signum(pivotError)); 
      reachMotor.set(reachOutput * Math.signum(reachError)); 
  }
  
  /**
   * Calculates the shortest angular distance between current and target positions.
   */
  private double getShortestPath(double currentAngle, double targetAngle) {
      double error = targetAngle - currentAngle;
      if (error > 180) {
          error -= 360;
      } else if (error < -180) {
          error += 360;
      }
      return error;

        //double liftOutput = liftPIDController.calculate(liftAngle);
        //liftMotor.set(liftOutput);

        //double pivotOutput = pivotPIDController.calculate(pivotAngle);
        //pivotMotor.set(pivotOutput);

        //double reachOutput = reachPIDController.calculate(reachAngle);
        //reachMotor.set(reachOutput);
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
