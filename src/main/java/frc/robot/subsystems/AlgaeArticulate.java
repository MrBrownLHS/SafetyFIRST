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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class AlgaeArticulate extends SubsystemBase {
    
    private final SparkMax m_AlgaeArticulate;
    private final SparkMaxConfig motorConfig;
    private final SlewRateLimiter algaeArticulateRateLimiter;


    public AlgaeArticulate() {
        m_AlgaeArticulate = new SparkMax(Constants.AlgaeCollectorConstants.ALGAE_ARTICULATE_MOTOR_ID, MotorType.kBrushless); 
        motorConfig = new SparkMaxConfig();
        algaeArticulateRateLimiter = new SlewRateLimiter(Constants.CollectorArmConstants.ARTICULATE_RATE_LIMIT);
        configureMotors(m_AlgaeArticulate, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
    }

    private void configureMotors(SparkMax motor, SparkMaxConfig config, int currentLimit) {
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(currentLimit);
      config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
      config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command AlgaeUpDown (DoubleSupplier joystickInput) {
        return new RunCommand(() -> {
          double rawInput = joystickInput.getAsDouble();
          double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
          double limitedInput = algaeArticulateRateLimiter.calculate(adjustedInput * 0.15);
          m_AlgaeArticulate.set(limitedInput);
        }, this);
    }

    public Command StopAlgaeArticulateMotor() {
        return new InstantCommand(() -> {
            m_AlgaeArticulate.set(0.0);
        }, this);
    }
    

    @Override
    public void periodic() {

    }


}