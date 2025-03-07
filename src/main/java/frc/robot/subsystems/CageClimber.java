// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CageClimber extends SubsystemBase {
  private final SparkMax winch1Motor, winch2Motor;
  private SparkMaxConfig winchMotorConfig;
 
  /** Creates a new CageClimber. */
  public CageClimber() {
    winch1Motor = new SparkMax(Constants.CageClimberConstants.WINCH_MOTOR_1_ID, MotorType.kBrushless);
    winch2Motor = new SparkMax(Constants.CageClimberConstants.WINCH_MOTOR_2_ID, MotorType.kBrushless);
    winchMotorConfig = new SparkMaxConfig();
        
    configureWinchMotor(winch1Motor, winchMotorConfig);
    configureWinchMotor(winch2Motor, winchMotorConfig);
  }

  private void configureWinchMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(Constants.CollectorArmConstants.DISABLE_NEUTRAL_MODE ? IdleMode.kCoast : IdleMode.kBrake);
    config.smartCurrentLimit(Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      
  }

  public void ReadyCageGrabber() {
    winch1Motor.set(0.25);
    winch2Motor.set(-0.25);

  }

  public void CageClimb() {
    winch1Motor.set(0.5);
    winch2Motor.set(-0.5);
  }

  public void CageClimbStop() {
    winch1Motor.set(0);
    winch2Motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
