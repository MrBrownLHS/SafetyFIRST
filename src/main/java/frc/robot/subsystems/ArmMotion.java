// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.utilities.constants.Constants;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.utilities.OnboardModuleState;
import frc.robot.utilities.constants.Constants;
import frc.robot.utilities.constants.SwerveModuleConstants;
/** Add your docs here. */
public class ArmMotion {
    private final SparkMax armMotor;
    private final RelativeEncoder armEncoder;
    private final PIDController armPIDController;
    private final CANcoder armEncoderCANcoder;
    private final CANcoderConfigurator armEncoderConfigurator;
    private final CANcoderConfiguration armEncoderConfig;
    private final MagnetSensorConfigs armMagnetSensorConfig;
    private final SensorDirectionValue armSensorDirection;
    private final PositionVoltage armPositionVoltage;

    public ArmMotion() {
        armMotor = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        armPIDController = new PIDController(
            Constants.ArmConstants.ARM_kP, 
            Constants.ArmConstants.ARM_kI, 
            Constants.ArmConstants.ARM_kD
        );
        armEncoderCANcoder = new CANcoder(Constants.ArmConstants.ARM_ENCODER_ID);
        armEncoderConfigurator = new CANcoderConfigurator(armEncoderCANcoder);
        armEncoderConfig = new CANcoderConfiguration();
        armMagnetSensorConfig = new MagnetSensorConfigs();
        armSensorDirection = new SensorDirectionValue();
        armPositionVoltage = new PositionVoltage();

        configureArmMotor();
        configureArmEncoder();
    }

    public void configureArmMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.setIdleMode(IdleMode.kBrake);
        config.setClosedLoopConfig(ClosedLoopSlot.kPrimary, 0.1, FeedbackSensor.kIntegratedSensor, ControlType.kPosition, 0.1, 0.1);
        config.setClosedLoopConfig(ClosedLoopSlot.kSecondary, 0.1, FeedbackSensor.kIntegratedSensor, ControlType.kVelocity, 0.1, 0.1);
        config.setOpenLoopRampRate(0.5);
        config.setClosedLoopRampRate(0.5);
        config.setSmartCurrentLimit(40);
        config.setSecondaryCurrentLimit(40);
        config.setPeakOutputForward(1);
        config.setPeakOutputReverse(-1);
        config.setAllowableClosedloopError(0, 10);

        armMotor.restoreFactoryDefaults();
        armMotor.configAllSettings(config);

        // Set the encoder to be the primary feedback sensor
        // This is done in the configureArmEncoder method
    }

    public void configureArmEncoder() {
        
    }
}
