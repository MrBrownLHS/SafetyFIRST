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
  private SparkClosedLoopController rotatePIDController;

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

    rotateMotorConfig.idleMode(IdleMode.kBrake);
    rotateMotorConfig.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);

    m_RotateMotor = new SparkMax(Constants.Rotate.ROTATE_MOTOR_ID, MotorType.kBrushless);
    rotateEncoder = m_RotateMotor.getEncoder();
    rotatePIDController = m_RotateMotor.getClosedLoopController();
    m_RotateMotor.configure(
        rotateMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    rotateProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            Constants.Rotate.ROTATE_MAX_VELOCITY,
            Constants.Rotate.ROTATE_MAX_ACCELERATION
        )
    );
  }

  public enum RotateState {
    COLLECT,
    L1,
    L2,
    L3,
    CLIMB
  }

  public static class PeriodicIO {
    double rotate_target = 0.0;
    double rotate_power = 0.0;

    boolean is_rotate_positional_control = true;

    RotateState state = RotateState.COLLECT;
  }

    
  // public RunCommand ArticulateCoralCollector(DoubleSupplier joystickInput) {
  //     return new RunCommand(() -> {
  //       double rawInput = joystickInput.getAsDouble();
  //       double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
  //       double limitedInput = articulateCollectorRateLimiter.calculate(adjustedInput);
  //       m_CoralArticulate.set(limitedInput);
  //     }, this);
  //   }
  
  // public RunCommand ArticulateCoralCollector(DoubleSupplier joystickInput) {
  //   return new RunCommand(() -> {
  //       double rawInput = joystickInput.getAsDouble();
  //       double adjustedInput = (Math.abs(rawInput) > 0.1)? rawInput : 0.0;

  //       // If adjusted input is 0.0, explicitly stop the motor
  //       if (adjustedInput == 0.0) {
  //           m_CoralArticulate.set(0.0);
  //       } else {
  //           double limitedInput = articulateCollectorRateLimiter.calculate(adjustedInput);
  //           m_CoralArticulate.set(limitedInput);
  //       }
  //   }, this);
  // }

  // public Command CollectorHeadStop() {
  //   return new InstantCommand(() -> {
  //     m_CoralArticulate.set(0.0);
  //   }, this);
  //   }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Rotate Position", rotateEncoder.getPosition());
    SmartDashboard.putBoolean("Rotate Positional Control", rotatePeriodicIO.is_rotate_positional_control);
    SmartDashboard.putNumber("Rotate Target Position", rotatePeriodicIO.rotate_target);
    SmartDashboard.putString("Rotate State", rotatePeriodicIO.state.toString());
    
  }
}
