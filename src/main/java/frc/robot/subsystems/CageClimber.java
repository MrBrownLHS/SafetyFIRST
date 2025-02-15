// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class CageClimber extends SubsystemBase {
  private final SparkMax Winch1Motor, Winch2Motor;
  private SparkMaxConfig WinchMotorConfig;
 
  /** Creates a new CageClimber. */
  public CageClimber() {
    Winch1Motor = new SparkMax(Constants.CageClimberConstants.WINCH_MOTOR_1_ID, MotorType.kBrushless);
    Winch2Motor = new SparkMax(Constants.CageClimberConstants.WINCH_MOTOR_2_ID, MotorType.kBrushless);
    WinchMotorConfig = new SparkMaxConfig();
    configureWinchMotor();
  }

  private void configureWinchMotor() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_CURRENT_LIMIT);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.COLLECTOR_ARM_MAX_CURRENT_LIMIT);
    config.voltageCompensation(Constants.CollectorArmConstants.COLLECTOR_ARM_VOLTAGE_COMPENSATION);
    Winch1Motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    Winch2Motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  
  }

  public void ReadyCageGrabber() {
    Winch1Motor.set(0.25);
    Winch2Motor.set(-0.25);

  }

  public void CageClimb() {
    Winch1Motor.set(0.5);
    Winch2Motor.set(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
