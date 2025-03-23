// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class AlgaeClaw extends SubsystemBase {

  private final SparkMax m_Claw;
  private final RelativeEncoder clawEncoder;
  private final SparkMaxConfig motorConfig;

  private static final double CLAW_OPEN = 0.0;
  private static final double CLAW_CLAMP = 90.0;
 
  public AlgaeClaw() {
    m_Claw = new SparkMax(Constants.AlgaeCollectorConstants.ALGAE_CLAW_MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    clawEncoder = m_Claw.getEncoder();

    configureMotors(m_Claw, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
    resetEncoder();
  }

 

  private void configureMotors(SparkMax motor, SparkMaxConfig config, int currentLimit) {
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(currentLimit);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void resetEncoder() {
    clawEncoder.setPosition(0.0);
  }

  public double getClawPosition() {
      return clawEncoder.getPosition();
  }
  public void openClaw() {
    m_Claw.set(0.3);
  }

  public void clampClaw() {
    Timer timeoutTimer = new Timer();
    timeoutTimer.start();
    m_Claw.set(-0.5);

    while (clawEncoder.getPosition() > CLAW_CLAMP) {
      if (timeoutTimer.hasElapsed(1.0)) { // Timeout after 1 second
          m_Claw.stopMotor();
      }

    }
    m_Claw.stopMotor();

  }


  public boolean isClamped() {
      return clawEncoder.getPosition() <= CLAW_CLAMP;
  }

  public boolean isOpen() {
      return clawEncoder.getPosition() >= CLAW_OPEN;
  }

  public void setPosition(double position) {
      if (position == CLAW_OPEN) {
          openClaw();
      } else if (position == CLAW_CLAMP) {
          clampClaw();
      }
  }

  public Command OpenClaw() {
        return new InstantCommand(() -> setPosition(CLAW_OPEN), this)
            .andThen(new WaitUntilCommand(this::isOpen))
            .andThen(new InstantCommand(this::stopClaw));
    }

  public Command ClampClaw() {
    return new InstantCommand(() -> setPosition(CLAW_CLAMP), this)
            .andThen(new WaitUntilCommand(this::isClamped))
            .andThen(new InstantCommand(this::stopClaw));
  }

  public RunCommand SimpleClawClose() {
    return new RunCommand(() -> {
      m_Claw.set(-0.10);
    }, this);
  }

  public RunCommand SimpleClawOpen() {
    return new RunCommand(() -> {
      m_Claw.set(0.10);
    }, this);
  }

  public Command stopClaw() {
    return new InstantCommand(() -> {
      m_Claw.stopMotor();
    }, this);
    }

 
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Claw Open", isOpen());
    SmartDashboard.putBoolean("Claw Clamped", isClamped());
    
  }
}
