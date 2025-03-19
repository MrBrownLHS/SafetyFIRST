// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class AlgaeArticulate extends SubsystemBase {
    
    private final SparkMax m_AlgaeArticulate;
    private final RelativeEncoder algaeArticulateEncoder;
    private final PIDController algaeArticulatePID;
    private final TrapezoidProfile.Constraints articulateConstraints;
    private TrapezoidProfile.State articulateGoal, articulateState;
    private final TrapezoidProfile algaeArticulateProfile;
    private final ArmFeedforward algaeArticulateFF;
    private final SparkMaxConfig motorConfig;
    
    private static final double UP_POSITION = 0.0;   // Start position (Up)
    private static final double OUT_POSITION = 110.0; // End position (Out)

    private static final double MAX_VELOCITY = 120.0; // Degrees per second
    private static final double MAX_ACCELERATION = 150.0; // Degrees per second²

    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.01;
    private static final double kG = 0.3;

    public AlgaeArticulate() {
        m_AlgaeArticulate = new SparkMax(Constants.AlgaeCollectorConstants.ALGAE_ARTICULATE_MOTOR_ID, MotorType.kBrushless); 
        motorConfig = new SparkMaxConfig();
        algaeArticulateEncoder = m_AlgaeArticulate.getEncoder();

        
        algaeArticulatePID = new PIDController(kP, kI, kD);
        algaeArticulatePID.setTolerance(2.0); // Within 2 degrees is "at target"

        algaeArticulateFF = new ArmFeedforward(0.1, kG, 0.02, 0.01);

        articulateConstraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
        algaeArticulateProfile = new TrapezoidProfile(articulateConstraints);

        articulateGoal = new TrapezoidProfile.State(UP_POSITION, 0);
        articulateState = new TrapezoidProfile.State(algaeArticulateEncoder.getPosition(), 0);

        SmartDashboard.putNumber("AlgaeArticulate kP", kP);
        SmartDashboard.putNumber("AlgaeArticulate kI", kI);
        SmartDashboard.putNumber("AlgaeArticulate kD", kD);
        SmartDashboard.putBoolean("AlgaeArticulate Tuning", false);

        configureMotors(m_AlgaeArticulate, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);

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
      algaeArticulateEncoder.setPosition(0.0);
  }
 

    public void setPosition(double targetDegrees) {
        targetDegrees = MathUtil.clamp(targetDegrees, UP_POSITION, OUT_POSITION);

        articulateGoal = new TrapezoidProfile.State(targetDegrees, 0);
        articulateState = algaeArticulateProfile.calculate(0.02, articulateState, articulateGoal);

        double pidOutput = algaeArticulatePID.calculate(algaeArticulateEncoder.getPosition(), articulateState.position);
        double feedforward = algaeArticulateFF.calculate(Math.toRadians(articulateState.position), 0.0);
        double motorOutput = pidOutput + feedforward;
        m_AlgaeArticulate.set(MathUtil.clamp(motorOutput, -1.0, 1.0));
    }

    public boolean isAtTarget() {
        return algaeArticulatePID.atSetpoint();
    }

    public void stop() {
        m_AlgaeArticulate.stopMotor();
    }

    public Command moveToUpPosition() {
        return new InstantCommand(() -> setPosition(UP_POSITION), this);
    }

    public Command moveToOutPosition() {
        return new InstantCommand(() -> setPosition(OUT_POSITION), this);
    }

    public Command manualControl(DoubleSupplier input) {
        return new RunCommand(() -> {
            double rawInput = input.getAsDouble();
            double limitedInput = MathUtil.clamp(rawInput, -1.0, 1.0);
            m_AlgaeArticulate.set(limitedInput * 0.5); // Limit manual speed
        }, this);
    }

    public Command stopMotor() {
        return new InstantCommand(this::stop, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AlgaeArticulate Position", algaeArticulateEncoder.getPosition());
        SmartDashboard.putBoolean("At Target", isAtTarget());

        if (SmartDashboard.getBoolean("AlgaeArticulate Tuning", false)) {
            algaeArticulatePID.setP(SmartDashboard.getNumber("AlgaeArticulate kP", kP));
            algaeArticulatePID.setI(SmartDashboard.getNumber("AlgaeArticulate kI", kI));
            algaeArticulatePID.setD(SmartDashboard.getNumber("AlgaeArticulate kD", kD));
        }
    
        
    }
}

