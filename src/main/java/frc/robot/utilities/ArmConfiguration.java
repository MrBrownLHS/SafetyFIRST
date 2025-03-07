// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.utilities.constants.Constants;
/** Add your docs here. */
public class ArmConfiguration {
    
    private CANcoder liftEncoder, pivotEncoder;
    private CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig;
    private MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig;
    private SparkMax liftMotor1, liftMotor2, pivotMotor1, pivotMotor2, topIntakeMotor, bottomIntakeMotor, articulateMotor;
    private SparkMaxConfig liftMotorConfig, pivotMotorConfig, articulateMotorConfig, topIntakeMotorConfig, bottomIntakeMotorConfig;
    

    public ArmConfiguration() {
        liftMotor1 = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_1_ID, MotorType.kBrushless);
        liftMotor2 = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_2_ID, MotorType.kBrushless);
        pivotMotor1 = new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_1_ID, MotorType.kBrushless);
        pivotMotor2 = new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_2_ID, MotorType.kBrushless);
        topIntakeMotor = new SparkMax(Constants.CollectorArmConstants.TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
        bottomIntakeMotor = new SparkMax(Constants.CollectorArmConstants.BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);
        articulateMotor = new SparkMax(Constants.CollectorArmConstants.ARTICULATE_MOTOR_ID, MotorType.kBrushless);
        
        // Apply motor configurations
        configureMotors();
    }

    private void configureMotors() {
        SparkMaxConfig liftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

        // Common configuration for lift and pivot motors
        configureMotor(liftMotor1, liftMotorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
        configureMotor(liftMotor2, liftMotorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
        configureMotor(pivotMotor1, pivotMotorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
        configureMotor(pivotMotor2, pivotMotorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);

        // Configure intake and articulation motors separately
        configureMotor(topIntakeMotor, intakeMotorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_550);
        configureMotor(bottomIntakeMotor, intakeMotorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_550);
        configureMotor(articulateMotor, intakeMotorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_550);
        
        // Set follower motors
        liftMotor2.set(liftMotor1.get());
        pivotMotor2.set(pivotMotor1.get());
    }

    private void configureMotor(SparkMax motor, SparkMaxConfig config, int currentLimit) {
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(currentLimit);
        config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
        config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
        }

    // Getter methods for CollectorArm to use these motors
    public SparkMax getLiftMotor1() { return liftMotor1; }
    public SparkMax getLiftMotor2() { return liftMotor2; }
    public SparkMax getPivotMotor1() { return pivotMotor1; }
    public SparkMax getPivotMotor2() { return pivotMotor2; }
    public SparkMax getTopIntakeMotor() { return topIntakeMotor; }
    public SparkMax getBottomIntakeMotor() { return bottomIntakeMotor; }
    public SparkMax getArticulateMotor() { return articulateMotor; }

       
    
        
        // Initialize encoders
        liftEncoder = new CANcoder(Constants.CollectorArmConstants.LIFT_ENCODER_ID);
        pivotEncoder = new CANcoder(Constants.CollectorArmConstants.PIVOT_ENCODER_ID);

        // Lift Encoder Configuration
        liftEncoderConfig = new CANcoderConfiguration();
        liftMagnetSensorConfig = new MagnetSensorConfigs();
        liftMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.ENCODER_TO_INCHES;
        liftMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        liftEncoderConfig.MagnetSensor = liftMagnetSensorConfig;
        liftEncoder.getConfigurator().apply(liftEncoderConfig);

        // Pivot Encoder Configuration
        pivotEncoderConfig = new CANcoderConfiguration();
        pivotMagnetSensorConfig = new MagnetSensorConfigs();
        pivotMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.ENCODER_TO_DEGREES;
        pivotMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoderConfig.MagnetSensor = pivotMagnetSensorConfig;
        pivotEncoder.getConfigurator().apply(pivotEncoderConfig);
    }

    public double getLiftHeightInches() {
        return liftEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_INCHES;
    }

    public double getPivotAngleDegrees() {
        return pivotEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_DEGREES;
    }

    public void updateDashboard(boolean tuningMode) {
        SmartDashboard.putNumber("Lift Height", getLiftHeightInches());
        SmartDashboard.putNumber("Pivot Angle", getPivotAngleDegrees());
        
        if (tuningMode) {
            SmartDashboard.putNumber("Lift Encoder Raw", liftEncoder.getAbsolutePosition().getValueAsDouble());
            SmartDashboard.putNumber("Pivot Encoder Raw", pivotEncoder.getAbsolutePosition().getValueAsDouble());
        }
    }

    public void logArmState() {
        DataLogManager.log("Lift Height: " + getLiftHeightInches());
        DataLogManager.log("Pivot Angle: " + getPivotAngleDegrees());
    }
}
}
