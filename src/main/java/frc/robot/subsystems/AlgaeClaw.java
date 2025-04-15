// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.RunCommand;





public class AlgaeClaw extends SubsystemBase {

  private final SparkMax m_Claw;
  private final SparkMaxConfig motorConfig;

  public AlgaeClaw() {
    m_Claw = new SparkMax(Constants.AlgaeCollectorConstants.ALGAE_CLAW_MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    
    configureMotors(m_Claw, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
    
  }

  private void configureMotors(SparkMax motor, SparkMaxConfig config, int currentLimit) {
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(currentLimit);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public RunCommand ClawClose() {
    return new RunCommand(() -> {
      m_Claw.set(-0.15);
    }, this);
  }

  public RunCommand ClawOpen() {
    return new RunCommand(() -> {
      m_Claw.set(0.15);
    }, this);
  }

  public Command StopClaw() {
    return new InstantCommand(() -> {
      m_Claw.set(0.0);
    }, this);
    }

 
  @Override
  public void periodic() {
      
  }
}
