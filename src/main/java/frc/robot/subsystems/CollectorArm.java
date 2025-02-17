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

import frc.robot.utilities.ArmState;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class CollectorArm extends SubsystemBase {
  private final SparkMax liftMotor, pivotMotor, topIntakeMotor, bottomIntakeMotor;
  private final CANcoder liftEncoder, pivotEncoder;
  private final PIDController liftPIDController, pivotPIDController;
  private final CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig;
  private final SparkMaxConfig liftMotorConfig, pivotMotorConfig, topIntakeMotorConfig, bottomIntakeMotorConfig;
  private final MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig;
  private final DigitalInput coralLimit, algaeLimit;
  private final ArmFeedforward liftFeedforward, pivotFeedforward;
  private final double liftOutput, pivotOutput;
  private final TrapezoidProfile.State liftGoal, pivotGoal;
  private final TrapezoidProfile.State liftCurrent, pivotCurrent;
  private final TrapezoidProfile.Constraints liftConstraints, pivotConstraints;

  public enum CollectorArmState {
    START, PICKUP, L1, L2, L3, MAX
  }




 
  public CollectorArm() { 
    // Declare motors
    liftMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
    liftMotorConfig = new SparkMaxConfig();
    liftEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_ID);
    pivotMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotMotorConfig = new SparkMaxConfig();
    pivotEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_ID);
    topIntakeMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    topIntakeMotorConfig = new SparkMaxConfig();
    bottomIntakeMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);
    bottomIntakeMotorConfig = new SparkMaxConfig();
    coralLimit = new DigitalInput(Constants.CollectorArmConstants.COLLECTOR_ARM_CORAL_LIMIT_ID);
    algaeLimit = new DigitalInput(Constants.CollectorArmConstants.COLLECTOR_ARM_ALGAE_LIMIT_ID);
    
    
    liftConstraints = new TrapezoidProfile.Constraints(
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_MAX_VELOCITY, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_MAX_ACCELERATION);
    pivotConstraints = new TrapezoidProfile.Constraints(
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_MAX_VELOCITY, 
      Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_MAX_ACCELERATION);
    
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

            // Configure PID (Tune these values!)
    configurePID(liftPIDController);
    configurePID(pivotPIDController);
    
    configureMotor(liftMotor, liftMotorConfig);
    configureMotor(pivotMotor, pivotMotorConfig);
    configureMotor(topIntakeMotor, topIntakeMotorConfig);
    configureMotor(bottomIntakeMotor, bottomIntakeMotorConfig);
    
    }

    public void updateDashboard() {
      SmartDashboard.putNumber("Lift Height", liftEncoder.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition().getValueAsDouble());
    }

    private void configureMotor(SparkMax motor, SparkMaxConfig config) {
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_CURRENT_LIMIT);
      config.secondaryCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_MAX_CURRENT_LIMIT);
      config.voltageCompensation(Constants.CollectorArmConstants.COLLECTOR_ARM_VOLTAGE_COMPENSATION);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void configurePID(PIDController pidController) {
      liftPIDController.setP(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kP);
      liftPIDController.setI(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kI);
      liftPIDController.setD(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_kD);
      pivotPIDController.setP(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kP);
      pivotPIDController.setI(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kI);
      pivotPIDController.setD(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_kD);
         
      
    }


    private double getShortestPath(double currentAngle, double targetAngle) {
      double error = targetAngle - currentAngle;
      if (error > 180) {
          error -= 360;
      } else if (error < -180) {
          error += 360;
      }
      return error;
    }

   
    private void setLiftPosition (double targetInches) {
      double currentInches = liftEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.COLLECTOR_ARM_DEGREES_TO_INCHES;
      double pidOutput = liftPIDController.calculate(currentInches, targetInches);
      double ffOutput = liftFeedforward.calculate(Math.toRadians(targetInches / Constants.CollectorArmConstants.COLLECTOR_ARM_DEGREES_TO_INCHES), 0);
      liftMotor.set(pidOutput + ffOutput);
    }

    private void setPivotPosition (double targetAngle) {
      double currentAngle = pivotEncoder.getAbsolutePosition().getValueAsDouble();
      double error = getShortestPath(currentAngle, targetAngle);
      double pidOutput = pivotPIDController.calculate(currentAngle, targetAngle);
      pivotMotor.set(pidOutput * Math.signum(error));
    }

    private ArmState currentState = ArmState.START; // Default state

    public void moveToState(ArmState state) {
        currentState = state;
    }

    public void updateLift() {
      switch (currentState) {
          case START:
              setLiftPosition(
                  Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_START);
              setPivotPosition( 
                  Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_START);
              break;
          case PICKUP:
              setLiftPosition(
                  Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_PICKUP);
              setPivotPosition( 
                  Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_PICKUP);
              break;
          case L1:
              setLiftPosition(
                  Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_L1);
              setPivotPosition( 
                  Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_L1);
              break;
          case L2:
              setLiftPosition(
                  Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_L2);
              setPivotPosition( 
                  Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_L2);
              break;
          case L3:
              setLiftPosition(
                  Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_L3);
              setPivotPosition( 
                  Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_L3);
              break;
          case MAX:
              setLiftPosition(
                  Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_MAX);
              setPivotPosition( 
                  Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_MAX);
              break;
      }
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
