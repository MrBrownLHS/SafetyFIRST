// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;



import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class AlgaeCollector extends SubsystemBase {
  private final SparkMax m_AlgaeArticulate;
  private final SparkMaxConfig algaeMotorConfig;
  
  /** Creates a new AlgaeCollector. */
  public AlgaeCollector() {
    m_AlgaeArticulate = new SparkMax(Constants.AlgaeCollectorConstants.ALGAE_ARTICULATE_MOTOR_ID, MotorType.kBrushless);

    algaeMotorConfig = new SparkMaxConfig();
    configureAlgaeMotor(m_AlgaeArticulate, algaeMotorConfig);

  }

  private void configureAlgaeMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public RunCommand AlgaeOut() {
      return new RunCommand(() -> {
        m_AlgaeArticulate.set(-0.5);
      }, this);
      }
  
  
  public RunCommand AlgaeIn() {
      return new RunCommand(() -> {
        m_AlgaeArticulate.set(0.5);
      }, this);
      }

  public RunCommand StopAlgae() {
    return new RunCommand(() -> {
      m_AlgaeArticulate.stopMotor();
    }, this);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
