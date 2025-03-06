// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//https://chatgpt.com/share/67b1689b-d7d8-800e-8877-33cd210398d7
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.CollectorArm.CollectorArmState;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation; 

public class CollectorArm extends SubsystemBase {
  private final SparkMax liftMotor1, liftMotor2, pivotMotor1, pivotMotor2, topIntakeMotor, bottomIntakeMotor, articulateMotor;
  private final CANcoder liftEncoder, pivotEncoder;
  private final PIDController liftPIDController, pivotPIDController;
  private final CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig;
  private final SparkMaxConfig liftMotorConfig, pivotMotorConfig, articulateMotorConfig, topIntakeMotorConfig, bottomIntakeMotorConfig;
  private final MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig;
  //private final DigitalInput coralLimit, algaeLimit;
  private ArmFeedforward liftFeedforward, pivotFeedforward;
  private TrapezoidProfile.Constraints liftConstraints, pivotConstraints;
  private TrapezoidProfile liftProfile, pivotProfile;
  private TrapezoidProfile.State liftGoal, pivotGoal, liftState, pivotState;
  private SlewRateLimiter rateLimiter;
  

  public enum CollectorArmState { //adjust angles as needed
    START(0, 0),
    FLOOR(5, -45),
    COLLECT(10, -30),
    L1(20, -15),
    L2(30, 0),
    L3(40, 15),
    MAX(50, 30);

    public final double liftHeightInches, pivotAngleDegrees;
    CollectorArmState(double liftHeight, double pivotAngle) {
        this.liftHeightInches = liftHeight;
        this.pivotAngleDegrees = pivotAngle;
      }
    
  }

  private CollectorArmState currentState = CollectorArmState.START; // Default state


  public CollectorArm() { 
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    
    DataLogManager.log("CollectorArm Subsystem Initialized");
    
    liftMotor1 = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_1_ID, MotorType.kBrushless);
    liftMotor2 = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_2_ID, MotorType.kBrushless);
    pivotMotor1 = new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_1_ID, MotorType.kBrushless);
    pivotMotor2 = new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_2_ID, MotorType.kBrushless);
    
    liftEncoder = new CANcoder(Constants.CollectorArmConstants.LIFT_ENCODER_ID);
    pivotEncoder = new CANcoder(Constants.CollectorArmConstants.PIVOT_ENCODER_ID);

    liftMotor2.set(liftMotor1.get());
    pivotMotor2.set(pivotMotor1.get());

    topIntakeMotor = new SparkMax(Constants.CollectorArmConstants.TOP_INTAKE_MOTOR_ID, MotorType.kBrushless);
    bottomIntakeMotor = new SparkMax(Constants.CollectorArmConstants.BOTTOM_INTAKE_MOTOR_ID, MotorType.kBrushless);
    articulateMotor = new SparkMax(Constants.CollectorArmConstants.ARTICULATE_MOTOR_ID, MotorType.kBrushless);
    
    rateLimiter = new SlewRateLimiter(Constants.CollectorArmConstants.ARTICULATE_RATE_LIMIT);

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


    liftEncoderConfig = new CANcoderConfiguration();
    liftMagnetSensorConfig = new MagnetSensorConfigs();
    liftMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.ENCODER_TO_INCHES;
    liftMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    liftEncoderConfig.MagnetSensor = liftMagnetSensorConfig;
    liftEncoder.getConfigurator().apply(liftEncoderConfig);

    pivotEncoderConfig = new CANcoderConfiguration();
    pivotMagnetSensorConfig = new MagnetSensorConfigs();
    pivotMagnetSensorConfig.MagnetOffset = Constants.CollectorArmConstants.ENCODER_TO_DEGREES;
    pivotMagnetSensorConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pivotEncoderConfig.MagnetSensor = pivotMagnetSensorConfig;
    pivotEncoder.getConfigurator().apply(pivotEncoderConfig);

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
    
    configureNEOMotor(liftMotor1, liftMotorConfig);
    configureNEOMotor(liftMotor2, liftMotorConfig);
    configureNEOMotor(articulateMotor, articulateMotorConfig);
    configureNEOMotor(pivotMotor1, pivotMotorConfig);
    configureNEOMotor(pivotMotor2, pivotMotorConfig);
    configure550Motor(topIntakeMotor, topIntakeMotorConfig);
    configure550Motor(bottomIntakeMotor, bottomIntakeMotorConfig);
    
    }

    private void configureNEOMotor(SparkMax motor, SparkMaxConfig config) {
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
      config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
      config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  
      }
    private void configure550Motor(SparkMax motor, SparkMaxConfig config) {
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(Constants.CollectorArmConstants.CURRENT_LIMIT_550);
      config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_550);
      config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
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
    //SmartDashboard.putBoolean("Algae Limit Switch", algaeLimit.get());
    //SmartDashboard.putBoolean("Coral Limit Switch", coralLimit.get());
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

 

  public double getLiftHeightInches() {
    return liftEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_INCHES;
}

  public double getPivotAngleDegrees() {
    return pivotEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_DEGREES;
}

  private CollectorArmState currentState = CollectorArmState.START; // Default state

  public void setLiftPosition(double targetInches) {
      TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Constants.CollectorArmConstants.LIFT_MAX_VELOCITY,
          Constants.CollectorArmConstants.LIFT_MAX_ACCELERATION
      );
      TrapezoidProfile.State liftState =
      new TrapezoidProfile.State(this.liftState.position, this.liftState.velocity);
      TrapezoidProfile liftProfile = new TrapezoidProfile(constraints);

      liftState = liftProfile.calculate(0.02, liftState, liftGoal);
      double pidOutput = liftPIDController.calculate(getLiftHeightInches(), liftState.position);
    
      double ffOutput = liftFeedforward.calculate(liftState.velocity, 0);
      liftMotor1.set(pidOutput + ffOutput);
      liftMotor2.set(-pidOutput + ffOutput);
}

  public void setPivotPosition(double targetAngle) {
      TrapezoidProfile.Constraints constraints = 
        new TrapezoidProfile.Constraints(
        Constants.CollectorArmConstants.PIVOT_MAX_VELOCITY,
        Constants.CollectorArmConstants.PIVOT_MAX_ACCELERATION
      );
      TrapezoidProfile.State pivotState =
        new TrapezoidProfile.State(targetAngle, 0);
      TrapezoidProfile pivotProfile = new TrapezoidProfile(constraints);
        
      pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);

      double pidOutput = pivotPIDController.calculate(getPivotAngleDegrees(), pivotState.position);
  
      double ffOutput = pivotFeedforward.calculate(pivotState.velocity, 0);
      pivotMotor1.set(pidOutput + ffOutput);
      pivotMotor2.set(-pidOutput + ffOutput);
}


    public void moveToState(CollectorArmState state) {
      setLiftPosition(state.liftHeightInches);
      setPivotPosition(state.pivotAngleDegrees);
}
    public boolean isAtTarget(CollectorArmState state) {
      return liftPIDController.atSetpoint() && pivotPIDController.atSetpoint();
    }

    public void stopArm() {
      liftMotor1.stopMotor();
      liftMotor2.stopMotor();
      pivotMotor1.stopMotor();
      pivotMotor2.stopMotor();
      topIntakeMotor.stopMotor();
      bottomIntakeMotor.stopMotor();
      articulateMotor.stopMotor();
      DataLogManager.log("Emergency Stop Triggered!");
  }

   
    public RunCommand ArticulateCollector(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = rateLimiter.calculate(adjustedInput);
        articulateMotor.set(limitedInput);
      }, this);
    }
  
    public RunCommand CollectAlgae(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = rateLimiter.calculate(adjustedInput);
        bottomIntakeMotor.set(-limitedInput); //May need to switch which motor is inverted so they run opposite directions
        topIntakeMotor.set(limitedInput);
      }, this);
    }

    public RunCommand CollectCoral(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        double rawInput = joystickInput.getAsDouble();
        double adjustedInput = (Math.abs(rawInput) > Constants.CollectorArmConstants.DEADBAND) ? rawInput : 0.0;
        double limitedInput = rateLimiter.calculate(adjustedInput);
        bottomIntakeMotor.set(limitedInput); 
      }, this);
    }

    public RunCommand YeetAlgae(DoubleSupplier joystickInput) {
      return new RunCommand(() -> {
        topIntakeMotor.set(Constants.CollectorArmConstants.YEET_SPEED);
        bottomIntakeMotor.set(Constants.CollectorArmConstants.YEET_SPEED);
      }, this);
    }

    public RunCommand AutoCoral() {
      return new RunCommand(() -> {
        bottomIntakeMotor.set(-0.5);
      }, this);
    }

  @Override
  public void periodic() {
    updateDashboard();
    logArmState();
    // This method will be called once per scheduler run
  }
}
