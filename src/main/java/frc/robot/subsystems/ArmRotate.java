// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.DoubleSupplier;


public class ArmRotate extends SubsystemBase {
  private SparkMax m_RotateMotor;
  private final SparkMaxConfig rotateMotorConfig;
  private final SlewRateLimiter rotateRateLimiter;

  public ArmRotate() {
    rotateRateLimiter = new SlewRateLimiter(Constants.Rotate.ROTATE_RATE_LIMIT);
    m_RotateMotor = new SparkMax(Constants.Rotate.ROTATE_MOTOR_ID, MotorType.kBrushless);
    rotateMotorConfig = new SparkMaxConfig();

    configureRotateMotor(m_RotateMotor, rotateMotorConfig);
  }

  private void configureRotateMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command CoralRotate (DoubleSupplier joystickInput) {
    return new RunCommand(() -> {
      double rawInput = joystickInput.getAsDouble();
      double adjustedInput = (Math.abs(rawInput) > Constants.Rotate.ROTATE_DEADBAND) ? rawInput : 0.0;
      double limitedInput = rotateRateLimiter.calculate(adjustedInput * 0.30); //increased from 0.25
      m_RotateMotor.set(limitedInput);
    }, this);
  }

  public Command CoralRotateAuto(double speed) {
    return new RunCommand(() -> {
            m_RotateMotor.set(speed);
        }, this).withTimeout(5.0
        
    );
  }

  public Command CoralRotateStop() {
    return new InstantCommand(() -> {
      m_RotateMotor.set(0.0);
    }, this);
  }
}
