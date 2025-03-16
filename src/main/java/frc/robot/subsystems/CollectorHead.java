// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;


import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.utilities.constants.Constants;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class CollectorHead extends SubsystemBase {
  private final SparkMax m_CoralArticulate;
  private final SparkMaxConfig articulateCollectorMotorConfig;
  public SlewRateLimiter articulateCollectorRateLimiter;
  
  public CollectorHead() {
    m_CoralArticulate = new SparkMax(Constants.CollectorArmConstants.CORAL_ARTICULATE_MOTOR_ID, MotorType.kBrushless);
    articulateCollectorRateLimiter = new SlewRateLimiter(Constants.CollectorArmConstants.ARTICULATE_RATE_LIMIT);
    articulateCollectorMotorConfig = new SparkMaxConfig();

    configureArticulateCollectorMotor(m_CoralArticulate, articulateCollectorMotorConfig);
  }

  private void configureArticulateCollectorMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public RunCommand ArticulateCoralCollector(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = articulateCollectorRateLimiter.calculate(adjustedInput);
        m_CoralArticulate.set(limitedInput);
      }, this);
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
