// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmRotate extends SubsystemBase {
  private PeriodicIO rotatePeriodicIO;
  private static ArmRotate rotateInstance;
  private static ArmRotate getInstance() {
    if (rotateInstance == null) {
      rotateInstance = new ArmRotate();
    }
    return rotateInstance;

  }
  private SparkMax m_RotateMotor;
  private RelativeEncoder rotateEncoder;
  private SparkClosedLoopController rotateController;

  private TrapezoidProfile rotateProfile;
  private TrapezoidProfile.State rotateCurrentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State rotateGoalState = new TrapezoidProfile.State();
  
  
  private ArmRotate() {
    super("ArmRotate");
    rotatePeriodicIO = new PeriodicIO();

    SparkMaxConfig rotateMotorConfig = new SparkMaxConfig();

    rotateMotorConfig.closedLoop
        .pid(Constants.Rotate.ROTATE_kP, Constants.Rotate.ROTATE_kI, Constants.Rotate.ROTATE_kD)
        .iZone(Constants.Rotate.ROTATE_kIZone);

    configureArticulateCollectorMotor(m_CoralArticulate, articulateCollectorMotorConfig);
  }

  private void configureArticulateCollectorMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // public RunCommand ArticulateCoralCollector(DoubleSupplier joystickInput) {
  //     return new RunCommand(() -> {
  //       double rawInput = joystickInput.getAsDouble();
  //       double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
  //       double limitedInput = articulateCollectorRateLimiter.calculate(adjustedInput);
  //       m_CoralArticulate.set(limitedInput);
  //     }, this);
  //   }
  
  public RunCommand ArticulateCoralCollector(DoubleSupplier joystickInput) {
    return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > 0.1)? rawInput : 0.0;

        // If adjusted input is 0.0, explicitly stop the motor
        if (adjustedInput == 0.0) {
            m_CoralArticulate.set(0.0);
        } else {
            double limitedInput = articulateCollectorRateLimiter.calculate(adjustedInput);
            m_CoralArticulate.set(limitedInput);
        }
    }, this);
  }

  public Command CollectorHeadStop() {
    return new InstantCommand(() -> {
      m_CoralArticulate.set(0.0);
    }, this);
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
