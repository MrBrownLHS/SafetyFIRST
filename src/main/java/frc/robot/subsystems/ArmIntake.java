// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ArmIntake extends SubsystemBase {
  private final SparkMax m_CoralCollect;
  private final SparkMaxConfig coralCollectorMotorConfig;
  public SlewRateLimiter coralCollectorRateLimiter;
  //private final DigitalInput limitSwitch;
  

 
    public ArmIntake() {
      coralCollectorRateLimiter = new SlewRateLimiter(Constants.Intake.CORAL_INTAKE_RATE_LIMIT);
      m_CoralCollect = new SparkMax(Constants.Intake.CORAL_INTAKE_MOTOR_ID, MotorType.kBrushless);
      coralCollectorMotorConfig = new SparkMaxConfig();
      //limitSwitch = new DigitalInput(Constants.Intake.CORAL_LIMIT_SWITCH_PORT); // Initialize the limit switch

      configureCoralCollectorMotor(m_CoralCollect, coralCollectorMotorConfig);
    }

    private void configureCoralCollectorMotor(SparkMax motor, SparkMaxConfig config){
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_550);
      config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_550);
      config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    // public void runCoralCollector(double speed){
    //   m_CoralCollect.set(speed);
    // }

    // public boolean isLimitSwitchPressed() {
    //   return !limitSwitch.get(); // Assuming the limit switch is active-low
    // }

    // public void runCoralCollector(double speed) {
    //   if (!isLimitSwitchPressed()) { // Only run if the limit switch is not pressed
    //       m_CoralCollect.set(speed);
    //   } else {
    //       m_CoralCollect.set(0.0); // Stop the motor if the limit switch is pressed
    //   }
    // }

    public RunCommand CoralIn() {
      return new RunCommand(() -> {
            m_CoralCollect.set(0.15);
      }, this);
    }

    public RunCommand CoralOut() {
      return new RunCommand(() -> {
          m_CoralCollect.set(-0.15); // Clamp to valid range
      }, this);
    }

  
    public RunCommand AutoCollectCoral() {
      return new RunCommand(() -> {
       m_CoralCollect.set(Constants.Intake.CORAL_AUTO_RELEASE_SPEED);
      }, this);
    }

    public Command CollectCoralStop() {
    return new InstantCommand(() -> {
      m_CoralCollect.set(0.0);
    }, this);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
