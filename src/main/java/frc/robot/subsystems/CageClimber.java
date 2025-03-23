// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class CageClimber extends SubsystemBase {
  private final SparkMax m_Winch;
  private SparkMaxConfig winchMotorConfig;
 
  /** Creates a new CageClimber. */
  public CageClimber() {
    m_Winch = new SparkMax(Constants.CageClimberConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
    
    winchMotorConfig = new SparkMaxConfig();
        
    configureWinchMotor(m_Winch, winchMotorConfig);
  }

  private void configureWinchMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      
  }

  public RunCommand ReadyCageGrabber() {
    return new RunCommand(() -> {
      m_Winch.set(-0.5);
    }, this);
    }


  public RunCommand CageClimb() {
    return new RunCommand(() -> {
      m_Winch.set(0.5);
    }, this);
    }

  public Command CageClimbStop() {
    return new InstantCommand(() -> {
      m_Winch.set(0.0);
    }, this);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
