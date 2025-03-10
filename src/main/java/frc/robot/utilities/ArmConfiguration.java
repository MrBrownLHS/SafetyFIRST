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
import frc.robot.utilities.ArmConfiguration;


/** Add your docs here. */
public class ArmConfiguration {
    
    public CANcoder liftEncoder, pivotEncoder;
    public CANcoderConfiguration liftEncoderConfig, pivotEncoderConfig;
    public MagnetSensorConfigs liftMagnetSensorConfig, pivotMagnetSensorConfig;
    public final SparkMax m_Lift, m_Pivot, m_CoralIntake, m_CoralArticulate;
    public final SparkMaxConfig motorConfig;
    private boolean tuningMode = false;
    public ArmFeedforward liftFeedforward, pivotFeedforward;
    public TrapezoidProfile.Constraints liftConstraints, pivotConstraints;
    public TrapezoidProfile liftProfile, pivotProfile;
    public TrapezoidProfile.State liftGoal, pivotGoal, liftState, pivotState, currentState;
    public SlewRateLimiter rateLimiter;
    public PIDController liftPIDController, pivotPIDController;
    

    public SparkMax [] getLiftMotor() { return new SparkMax[]{m_Lift}; }
    public SparkMax [] getPivotMotor() { return new SparkMax[]{m_Pivot}; }
    public SparkMax [] getIntakeMotor() { return new SparkMax[]{m_CoralIntake}; }
    public SparkMax getArticulateMotor() { return m_CoralArticulate; }
    public CANcoder getLiftEncoder() { return liftEncoder; }
    public CANcoder getPivotEncoder() { return pivotEncoder; }
    public TrapezoidProfile.State getLiftGoal() { return liftGoal; }
    public TrapezoidProfile.State getPivotGoal() { return pivotGoal; }
    public TrapezoidProfile.State getLiftState() { return liftState; }
    public TrapezoidProfile.State getPivotState() { return pivotState; }
    public TrapezoidProfile.State getCurrentState () { return currentState; }
    public TrapezoidProfile.Constraints getLiftConstraints() { return liftConstraints; }
    public TrapezoidProfile.Constraints getPivotConstraints() { return pivotConstraints; }
    public ArmFeedforward getLiftFeedforward() { return liftFeedforward; }
    public ArmFeedforward getPivotFeedforward() { return pivotFeedforward; }
    public PIDController getLiftPIDController() { return liftPIDController; }
    public PIDController getPivotPIDController() { return pivotPIDController; }
    public boolean isTuningMode() { return tuningMode; }
    public SlewRateLimiter getRateLimiter() { return rateLimiter; }
    
    
    public ArmConfiguration() {
        m_Lift = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_ID, MotorType.kBrushless);
        m_Pivot= new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        m_CoralArticulate = new SparkMax(Constants.CollectorArmConstants.CORAL_INTAKE_MOTOR_ID, MotorType.kBrushless);
        m_CoralIntake = new SparkMax(Constants.CollectorArmConstants.CORAL_ARTICULATE_MOTOR_ID, MotorType.kBrushless);

        liftEncoder = new CANcoder(Constants.CollectorArmConstants.LIFT_ENCODER_ID);
        pivotEncoder = new CANcoder(Constants.CollectorArmConstants.PIVOT_ENCODER_ID);

        liftPIDController = new PIDController(Constants.CollectorArmConstants.LIFT_kP, Constants.CollectorArmConstants.LIFT_kI, Constants.CollectorArmConstants.LIFT_kD);
        pivotPIDController = new PIDController(Constants.CollectorArmConstants.PIVOT_kP, Constants.CollectorArmConstants.PIVOT_kI, Constants.CollectorArmConstants.PIVOT_kD);
        
        motorConfig = new SparkMaxConfig();
       
        configureMotors();
        configureEncoders();
        configurePIDControllers(liftPIDController, Constants.CollectorArmConstants.LIFT_kP, Constants.CollectorArmConstants.LIFT_kI, Constants.CollectorArmConstants.LIFT_kD);
        configurePIDControllers(pivotPIDController, Constants.CollectorArmConstants.PIVOT_kP, Constants.CollectorArmConstants.PIVOT_kI, Constants.CollectorArmConstants.PIVOT_kD);
        
    }
   //Need to invert lift and pivot motors: https://www.reddit.com/r/FRC/comments/1id6sz2/how_to_invert_a_spark_max/
    private void configureMotors() {
        SparkMax[] neoMotors = {m_Lift, m_Pivot, m_CoralArticulate};
        SparkMax[] neo550Motors = {m_CoralIntake, m_CoralArticulate};

        for (SparkMax motor : neoMotors) {
            configureNEOMotor(motor, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
        }

        for (SparkMax motor : neo550Motors) {
            configure550Motor(motor, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_550);
        }

    }

    private void configureNEOMotor(SparkMax neoMotors, SparkMaxConfig config, int currentLimit) {
        config.idleMode(Constants.CollectorArmConstants.DISABLE_NEUTRAL_MODE ? IdleMode.kCoast : IdleMode.kBrake);
        config.smartCurrentLimit(currentLimit);
        config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
        config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
        neoMotors.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configure550Motor(SparkMax neo550motor, SparkMaxConfig config, int currentLimit) {
        config.idleMode(Constants.CollectorArmConstants.DISABLE_NEUTRAL_MODE ? IdleMode.kCoast : IdleMode.kBrake);
        config.smartCurrentLimit(currentLimit);
        config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_550);
        config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
        neo550motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureEncoders() {
        configureSingleEncoder(liftEncoder, Constants.CollectorArmConstants.ENCODER_TO_INCHES);
        configureSingleEncoder(pivotEncoder, Constants.CollectorArmConstants.ENCODER_TO_DEGREES);
    }

    private void configureSingleEncoder(CANcoder encoder, double magnetOffset) {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        magnetConfig.MagnetOffset = magnetOffset;
        magnetConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor = magnetConfig;
        encoder.getConfigurator().apply(encoderConfig);
    }

    private void configurePIDControllers(PIDController pidController, double kP, double kI, double kD) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
     

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
  
      liftGoal = new TrapezoidProfile.State(0, 0);
      liftState = liftProfile.calculate(0.02, liftState, liftGoal);

      pivotGoal = new TrapezoidProfile.State(0, 0);
      pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);
  
      liftProfile = new TrapezoidProfile(liftConstraints);
      pivotProfile = new TrapezoidProfile(pivotConstraints);
              
      configurePIDControllers(liftPIDController, Constants.CollectorArmConstants.LIFT_kP,
      Constants.CollectorArmConstants.LIFT_kI, Constants.CollectorArmConstants.LIFT_kD);
  
      configurePIDControllers(pivotPIDController, Constants.CollectorArmConstants.PIVOT_kP,
      Constants.CollectorArmConstants.PIVOT_kI, Constants.CollectorArmConstants.PIVOT_kD);
    }

 
    public void updatePIDTuning(boolean tuningMode) {
        this.tuningMode = tuningMode;
        SmartDashboard.putBoolean("Tuning Mode", tuningMode);
        
        if (tuningMode) {
            updateSmartDashboardPID("Lift", liftPIDController);
            updateSmartDashboardPID("Pivot", pivotPIDController);
    
            double newLiftKg = SmartDashboard.getNumber("Lift kG", liftFeedforward.getKg());
            double newPivotKg = SmartDashboard.getNumber("Pivot kG", pivotFeedforward.getKg());
    
            // Only update if values change to reduce CPU load
            if (newLiftKg != liftFeedforward.getKg()) {
                liftFeedforward = new ArmFeedforward(
                    SmartDashboard.getNumber("Lift kS", liftFeedforward.getKs()), 
                    newLiftKg, 
                    SmartDashboard.getNumber("Lift kV", liftFeedforward.getKv()), 
                    SmartDashboard.getNumber("Lift kA", liftFeedforward.getKa()));
            }
    
            if (newPivotKg != pivotFeedforward.getKg()) {
                pivotFeedforward = new ArmFeedforward(
                    SmartDashboard.getNumber("Pivot kS", pivotFeedforward.getKs()), 
                    newPivotKg, 
                    SmartDashboard.getNumber("Pivot kV", pivotFeedforward.getKv()), 
                    SmartDashboard.getNumber("Pivot kA", pivotFeedforward.getKa()));
            }


                SmartDashboard.putNumber("Lift Encoder Raw", liftEncoder.getAbsolutePosition().getValueAsDouble());
                SmartDashboard.putNumber("Pivot Encoder Raw", pivotEncoder.getAbsolutePosition().getValueAsDouble());
                SmartDashboard.putNumber("Lift Height", liftEncoder.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("Lift Setpoint", liftPIDController.getSetpoint());
                SmartDashboard.putNumber("Pivot Setpoint", pivotPIDController.getSetpoint());
                SmartDashboard.putNumber("Lift PID Error", liftPIDController.getPositionError());
                SmartDashboard.putNumber("Pivot PID Error", pivotPIDController.getPositionError());
                SmartDashboard.putNumber("Lift Motor Output", m_Lift.get());
                SmartDashboard.putNumber("Pivot Motor Output", m_Pivot.get());
                SmartDashboard.putNumber("Articulate Motor Output", m_CoralArticulate.get());
                
            
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

        }
    }  

    private void updateSmartDashboardPID(String name, PIDController controller) {
                SmartDashboard.putNumber(name + " kP", controller.getP());
                controller.setP(SmartDashboard.getNumber(name + " kP", controller.getP()));
                
                SmartDashboard.putNumber(name + " kI", controller.getI());
                controller.setI(SmartDashboard.getNumber(name + " kI", controller.getI()));

                SmartDashboard.putNumber(name + " kD", controller.getD());
                controller.setD(SmartDashboard.getNumber(name + " kD", controller.getD()));
    }


    public void logArmState() {
                DataLogManager.log("Current State: Position=" + currentState.position + ", Velocity=" + currentState.velocity);
                DataLogManager.log("Lift Height: " + liftEncoder.getPosition().getValueAsDouble());
                DataLogManager.log("Pivot Angle: " + pivotEncoder.getPosition().getValueAsDouble());
                DataLogManager.log("Lift Output 1: " + m_Lift.get());
                DataLogManager.log("Pivot Output 1: " + m_Pivot.get());
                DataLogManager.log("Articulate Output: " + m_CoralArticulate.get());
    
    }

}


