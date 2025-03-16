// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class AlgaeCollector extends SubsystemBase {
  private final SparkMax m_AlgaeArticulate;// m_AlgaeIntake1, m_AlgaeIntake2;
  private final SparkMaxConfig algaeMotorConfig;
  public SlewRateLimiter algaeRateLimiter;
  /** Creates a new AlgaeCollector. */
  public AlgaeCollector() {
    m_AlgaeArticulate = new SparkMax(Constants.AlgaeCollectorConstants.ALGAE_ARTICULATE_MOTOR_ID, MotorType.kBrushless);
    //m_AlgaeIntake1 = new SparkMax(Constants.AlgaeCollectorConstants.ALGAE_INTAKE_MOTOR_1_ID, MotorType.kBrushless);
   // m_AlgaeIntake2 = new SparkMax(Constants.AlgaeCollectorConstants.ALGAE_INTAKE_MOTOR_2_ID, MotorType.kBrushless);
    algaeRateLimiter = new SlewRateLimiter(Constants.AlgaeCollectorConstants.ALGAE_RATE_LIMIT);

    algaeMotorConfig = new SparkMaxConfig();
    configureAlgaeMotor(m_AlgaeArticulate, algaeMotorConfig);
    //configureAlgaeMotor(m_AlgaeIntake1, algaeMotorConfig);
    //configureAlgaeMotor(m_AlgaeIntake2, algaeMotorConfig);
  }

  private void configureAlgaeMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.CollectorArmConstants.CURRENT_LIMIT_550);
    config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_550);
    config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public RunCommand ArticulateAlgaeCollector(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = algaeRateLimiter.calculate(adjustedInput);
        m_AlgaeArticulate.set(limitedInput); 
      }, this);
    }
  
  //public RunCommand CollectAlgae(DoubleSupplier joystickInput) {
      //return new RunCommand(() -> {
        //double limitedInput = joystickInput.getAsDouble();
        //AlgaeIntake1.set(rawInput);
        //AlgaeIntake2.set(-rawInput);
      //}, this);
   // }

  public RunCommand ArticulateAndCollectAlgae(DoubleSupplier articulate, DoubleSupplier collect) {
    return new RunCommand(() -> {
      double rawInputArticulate = articulate.getAsDouble();
      double adjustedInput = (Math.abs(rawInputArticulate) > Constants.CollectorArmConstants.DEADBAND) ? rawInputArticulate : 0.0;
      double limitedInput = algaeRateLimiter.calculate(adjustedInput);
      m_AlgaeArticulate.set(limitedInput);

      double rawInputCollect = collect.getAsDouble();
      //AlgaeIntake1.set(rawInputCollect);
      //AlgaeIntake2.set(-rawInputCollect);
    }, this);
  }

  public RunCommand StopAlgae() {
    return new RunCommand(() -> {
      //AlgaeIntake1.stopMotor();
      //AlgaeIntake2.stopMotor();
      m_AlgaeArticulate.stopMotor();
    }, this);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
