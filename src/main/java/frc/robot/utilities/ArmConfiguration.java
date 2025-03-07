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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.utilities.constants.Constants;
/** Add your docs here. */
public class ArmConfiguration {
    
    private CANcoder liftEncoder, pivotEncoder;
    private CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig;
    private MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig;
    private SparkMax liftMotor1, liftMotor2, pivotMotor1, pivotMotor2, topIntakeMotor, bottomIntakeMotor, articulateMotor;
    private SparkMaxConfig liftMotorConfig, pivotMotorConfig, articulateMotorConfig, topIntakeMotorConfig, bottomIntakeMotorConfig;
    private final ArmConfiguration armconfig;
    private boolean tuningMode = false;
    private ArmFeedforward liftFeedforward, pivotFeedforward;
    private TrapezoidProfile.Constraints liftConstraints, pivotConstraints;
    private TrapezoidProfile liftProfile, pivotProfile;
    private TrapezoidProfile.State liftGoal, pivotGoal, liftState, pivotState;
    private SlewRateLimiter rateLimiter;
    private final PIDController liftPIDController, pivotPIDController;

    public SparkMax getLiftMotor1() { return liftMotor1; }
    public SparkMax getLiftMotor2() { return liftMotor2; }
    public SparkMax getPivotMotor1() { return pivotMotor1; }
    public SparkMax getPivotMotor2() { return pivotMotor2; }
    public SparkMax getTopIntakeMotor() { return topIntakeMotor; }
    public SparkMax getBottomIntakeMotor() { return bottomIntakeMotor; }
    public SparkMax getArticulateMotor() { return articulateMotor; }
    public CANcoder getLiftEncoder() { return liftEncoder; }
    public CANcoder getPivotEncoder() { return pivotEncoder; }
    public TrapezoidProfile.State getLiftGoal() { return liftGoal; }
    public TrapezoidProfile.State getPivotGoal() { return pivotGoal; }
    public TrapezoidProfile.State getLiftState() { return liftState; }
    public TrapezoidProfile.State getPivotState() { return pivotState; }
    public TrapezoidProfile.Constraints getLiftConstraints() { return liftConstraints; }
    public TrapezoidProfile.Constraints getPivotConstraints() { return pivotConstraints; }
    public ArmFeedforward getLiftFeedforward() { return liftFeedforward; }
    public ArmFeedforward getPivotFeedforward() { return pivotFeedforward; }
    public PIDController getLiftPIDController() { return liftPIDController; }
    public PIDController getPivotPIDController() { return pivotPIDController; }
    
    

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

    liftConstraints = new TrapezoidProfile.Constraints(
        Constants.CollectorArmConstants.LIFT_MAX_VELOCITY, 
        Constants.CollectorArmConstants.LIFT_MAX_ACCELERATION
      );
  
      pivotConstraints = new TrapezoidProfile.Constraints(
        Constants.CollectorArmConstants.PIVOT_MAX_VELOCITY, 
        Constants.CollectorArmConstants.PIVOT_MAX_ACCELERATION
      );
  
      liftState = new TrapezoidProfile.State(0, 0);
      pivotState = new TrapezoidProfile.State(0, 0);
  
      liftGoal = new TrapezoidProfile.State(0, 0);
      pivotGoal = new TrapezoidProfile.State(0, 0);
  
      liftFeedforward = new ArmFeedforward(Constants.CollectorArmConstants.LIFT_kS, 
                                          Constants.CollectorArmConstants.LIFT_kG, 
                                          Constants.CollectorArmConstants.LIFT_kV, 
                                          Constants.CollectorArmConstants.LIFT_kA);
  
      pivotFeedforward = new ArmFeedforward(Constants.CollectorArmConstants.PIVOT_kS, 
                                          Constants.CollectorArmConstants.PIVOT_kG, 
                                          Constants.CollectorArmConstants.PIVOT_kV, 
                                          Constants.CollectorArmConstants.PIVOT_kA);
  
      liftPIDController = new PIDController(Constants.CollectorArmConstants.LIFT_kP, 
                                          Constants.CollectorArmConstants.LIFT_kI, 
                                          Constants.CollectorArmConstants.LIFT_kD);
      liftPIDController.setTolerance(Constants.CollectorArmConstants.LIFT_TOLERANCE);
  
      pivotPIDController = new PIDController(Constants.CollectorArmConstants.PIVOT_kP, 
                                            Constants.CollectorArmConstants.PIVOT_kI,
                                            Constants.CollectorArmConstants.PIVOT_kD);
      pivotPIDController.setTolerance(Constants.CollectorArmConstants.PIVOT_TOLERANCE);
  
  
  
      TrapezoidProfile.State liftGoal = new TrapezoidProfile.State(0, 0);
        liftState = liftProfile.calculate(0.02, liftState, liftGoal);
      TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State(0, 0);
        pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);
  
      liftProfile = new TrapezoidProfile(liftConstraints);
      pivotProfile = new TrapezoidProfile(pivotConstraints);
              
             
          // Set encoder conversion factor to return values in degrees
     
  
              // Configure PID (Tune these values!)
      configurePID(liftPIDController, Constants.CollectorArmConstants.LIFT_kP,
        Constants.CollectorArmConstants.LIFT_kI, Constants.CollectorArmConstants.LIFT_kD);
  
      configurePID(pivotPIDController, Constants.CollectorArmConstants.PIVOT_kP,
        Constants.CollectorArmConstants.PIVOT_kI, Constants.CollectorArmConstants.PIVOT_kD);
      }
  
    
      private void configurePID(PIDController pidController, double kP, double kI, double kD) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
      }

    public void updateDashboard() {
        SmartDashboard.putNumber("Lift Height", liftEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Lift Setpoint", liftPIDController.getSetpoint());
        SmartDashboard.putNumber("Pivot Setpoint", pivotPIDController.getSetpoint());
        SmartDashboard.putNumber("Lift PID Error", liftPIDController.getPositionError());
        SmartDashboard.putNumber("Pivot PID Error", pivotPIDController.getPositionError());
        SmartDashboard.putNumber("Lift Motor 1 Output", liftMotor1.get());
        SmartDashboard.putNumber("Lift Motor 2 Output", liftMotor2.get());
        SmartDashboard.putNumber("Pivot Motor 1 Output", pivotMotor1.get());
        SmartDashboard.putNumber("Pivot Motor 2 Output", pivotMotor2.get());
        SmartDashboard.putNumber("Articulate Motor Output", articulateMotor.get());
        SmartDashboard.putString("Current Arm State", currentState.name());
    
        SmartDashboard.putNumber("Lift kP", liftPIDController.getP());
        liftPIDController.setP(SmartDashboard.getNumber("Lift kP", 0.05));
        SmartDashboard.putNumber("Lift kI", liftPIDController.getI());
        liftPIDController.setI(SmartDashboard.getNumber("Lift kI", 0.0));
        SmartDashboard.putNumber("Lift kD", liftPIDController.getD());
        liftPIDController.setD(SmartDashboard.getNumber("Lift kD", 0.0));
        SmartDashboard.putNumber("Pivot kP", pivotPIDController.getP());
        pivotPIDController.setP(SmartDashboard.getNumber("Pivot kP", 0.05));
        SmartDashboard.putNumber("Pivot kI", pivotPIDController.getI());
        pivotPIDController.setI(SmartDashboard.getNumber("Pivot kI", 0.0));
        SmartDashboard.putNumber("Pivot kD", pivotPIDController.getD());
        pivotPIDController.setD(SmartDashboard.getNumber("Pivot kD", 0.0));

        SmartDashboard.putNumber("Lift kG", liftFeedforward.getKg());
        liftFeedforward = new ArmFeedforward(0.1, SmartDashboard.getNumber("Lift kG", 1.0), 0.2, 0.05);
        SmartDashboard.putNumber("Pivot kG", pivotFeedforward.getKg());
        pivotFeedforward = new ArmFeedforward(0.1, SmartDashboard.getNumber("Pivot kG", 1.0), 0.2, 0.05);
        SmartDashboard.putNumber("Lift kS", liftFeedforward.getKs());
        liftFeedforward = new ArmFeedforward(SmartDashboard.getNumber("Lift kS", 0.1), 1.0, 0.2, 0.05);
        SmartDashboard.putNumber("Pivot kS", pivotFeedforward.getKs());
        pivotFeedforward = new ArmFeedforward(SmartDashboard.getNumber("Pivot kS", 0.1), 1.0, 0.2, 0.05);
        SmartDashboard.putNumber("Lift kV", liftFeedforward.getKv());
        liftFeedforward = new ArmFeedforward(0.1, 1.0, SmartDashboard.getNumber("Lift kV", 0.2), 0.05);
        SmartDashboard.putNumber("Pivot kV", pivotFeedforward.getKv());
        pivotFeedforward = new ArmFeedforward(0.1, 1.0, SmartDashboard.getNumber("Pivot kV", 0.2), 0.05);
        SmartDashboard.putNumber("Lift kA", liftFeedforward.getKa());
        liftFeedforward = new ArmFeedforward(0.1, 1.0, 0.2, SmartDashboard.getNumber("Lift kA", 0.05));
        SmartDashboard.putNumber("Pivot kA", pivotFeedforward.getKa());
        pivotFeedforward = new ArmFeedforward(0.1, 1.0, 0.2, SmartDashboard.getNumber("Pivot kA", 0.05));
    

  }

  public void logArmState() {
    DataLogManager.log("Current State: " + currentState.name());
    DataLogManager.log("Lift Height: " + liftEncoder.getPosition().getValueAsDouble());
    DataLogManager.log("Pivot Angle: " + pivotEncoder.getPosition().getValueAsDouble());
    DataLogManager.log("Lift Output 1: " + liftMotor1.get());
    DataLogManager.log("Lift Output 2: " + liftMotor2.get());
    DataLogManager.log("Pivot Output 1: " + pivotMotor1.get());
    DataLogManager.log("Pivot Output 2: " + pivotMotor2.get());
}

 


    public void logArmState() {
        DataLogManager.log("Lift Height: " + getLiftHeightInches());
        DataLogManager.log("Pivot Angle: " + getPivotAngleDegrees());
    }
}

