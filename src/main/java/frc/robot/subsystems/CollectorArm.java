// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.utilities.constants.Constants;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class CollectorArm extends SubsystemBase {
  private final SparkMax liftMotor, pivotMotor, reachMotor, topIntakeMotor, bottomIntakeMotor;
  private final CANcoder liftEncoder, pivotEncoder, reachEncoder;
  private final PIDController liftPIDController, pivotPIDController, reachPIDController;
  private final CANcoderConfigurator liftEncoderConfigurator, pivotEncoderConfigurator, reachEncoderConfigurator;
  private final CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig, reachEncoderConfig;
  private final MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig, reachMagnetSensorConfig;
  private final SensorDirectionValue liftSensorDirection, pivotSensorDirection, reachSensorDirection;
  private final PositionVoltage liftPositionVoltage, pivotPositionVoltage, reachPositionVoltage;

  
 
 

  public CollectorArm() {
    liftMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
    liftEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_ENCODER_ID);
    pivotMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_ENCODER_ID);
    reachMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_MOTOR_ID, MotorType.kBrushless);
    reachEncoder = new CANcoder(Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_ENCODER_ID);
    topIntakeMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    bottomIntakeMotor = new SparkMax(Constants.CollectorArmConstants.COLLECTOR_ARM_BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);


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

    liftEncoderConfig = new CANcoderConfiguration();
    pivotEncoderConfig = new CANcoderConfiguration();
    reachEncoderConfig = new CANcoderConfiguration();

      
  }

  private void configureCollectorArmEncoder() {

  }

  public void setArmPosition(double liftTarget, double pivotTarget, double reachTarget) {
    StatusSignal<Angle> liftAngleSignal = liftEncoder.getPosition();
    double currentLiftAngle = liftAngleSignal.getValue().toDegrees();

    double LiftOutput = liftPIDController.calculate(currentLiftAngle, liftTarget);
    liftMotor.set(LiftOutput);

    StatusSignal<Angle> pivotAngle = pivotEncoder.getPosition();
    StatusSignal<Angle> reachAngle = reachEncoder.getPosition();

    double liftOutput = liftPIDController.calculate(liftAngle.getPosition(), liftTarget);
    double liftOutput = liftPIDController.calculate(liftEncoder.getPosition(), Constants.CollectorArmConstants.COLLECTOR_ARM_LIFT_SETPOINT);
    double pivotOutput = pivotPIDController.calculate(pivotEncoder.getPosition(), Constants.CollectorArmConstants.COLLECTOR_ARM_PIVOT_SETPOINT);
    double reachOutput = reachPIDController.calculate(reachEncoder.getPosition(), Constants.CollectorArmConstants.COLLECTOR_ARM_REACH_SETPOINT);

    liftMotor.setVoltage(liftOutput);
    pivotMotor.setVoltage(pivotOutput);
    reachMotor.setVoltage(reachOutput);

    return 0;
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
