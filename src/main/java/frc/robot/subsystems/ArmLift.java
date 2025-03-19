package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;


public class ArmLift extends SubsystemBase {
    private final SparkMax m_Lift;
    private final RelativeEncoder liftEncoder;
    private final SparkMaxConfig motorConfig;
    private final PIDController liftPID;
    private final TrapezoidProfile.Constraints liftConstraints;
    private TrapezoidProfile.State liftGoal, liftState;
    private final TrapezoidProfile liftProfile;
    private final ArmFeedforward liftFF;

    private static final double LIFT_INCREMENT = 1.0;  // Inches per button press
    private static final double JOYSTICK_DEADBAND = 0.05; // Ignore small joystick movements

    private static final double LIFT_MAX_VELOCITY = 1.0;
    private static final double LIFT_MAX_ACCELERATION = 10;

    private static final double LIFT_MAX_HEIGHT = 10;
    private static final double LIFT_MIN_HEIGHT = 0;

    private static final double LIFT_GEAR_RATIO = 84.0;
    private static final double COG_DIAMETER_INCHES = 2.0;
    private static final double LIFT_ENCODER_TO_INCHES = (Math.PI * COG_DIAMETER_INCHES) / LIFT_GEAR_RATIO;

    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.01;
    private static final double kG = 0.3;

    public ArmLift() {
      m_Lift = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_ID, MotorType.kBrushless);
      liftEncoder = m_Lift.getEncoder();
      motorConfig = new SparkMaxConfig();

      liftPID = new PIDController(kP, kI, kD);
      liftPID.setTolerance(2.0);

      liftFF = new ArmFeedforward(0.1, kG, 0.02, 0.01);

      liftConstraints = new TrapezoidProfile.Constraints(LIFT_MAX_VELOCITY, LIFT_MAX_ACCELERATION);
      liftProfile = new TrapezoidProfile(liftConstraints);

      liftGoal = new TrapezoidProfile.State(LIFT_MAX_HEIGHT, 0);
      liftState = new TrapezoidProfile.State(liftEncoder.getPosition(), 0);

        SmartDashboard.putNumber("Lift kP", kP);
        SmartDashboard.putNumber("Lift kI", kI);
        SmartDashboard.putNumber("Lift kD", kD);
        SmartDashboard.putBoolean("Lift Tuning", false);


      configureMotors(m_Lift, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
      resetEncoder();
    }
   
    private void configureMotors(SparkMax motor, SparkMaxConfig config, int currentLimit) {
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(currentLimit);
        config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
        config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void resetEncoder(){
        liftEncoder.setPosition(0.0);
    }

    public void setPosition(double targetInches) {
        targetInches = MathUtil.clamp(targetInches, LIFT_MAX_HEIGHT, LIFT_MIN_HEIGHT);

        liftGoal = new TrapezoidProfile.State(targetInches, 0);
        liftState = liftProfile.calculate(0.02, liftState, liftGoal);

        double pidOutput = liftPID.calculate(liftEncoder.getPosition(), liftState.position);
        double feedforward = liftFF.calcualte(Math.to) //finish here based on AlgaeArticulate

    }

    
    public double getLiftHeightInches() {
        return liftEncoder.getAbsolutePosition().getValueAsDouble()* Constants.CollectorArmConstants.ENCODER_TO_INCHES;
    }

    public void setLiftPower(double power) {
        double currentHeight = getLiftHeightInches();
        double safePower = MathUtil.clamp(power, -1.0, 1.0);

        // Prevent movement beyond limits
        if ((safePower < 0 && currentHeight <= Constants.CollectorArmConstants.LIFT_MIN_HEIGHT) || 
            (safePower > 0 && currentHeight >= Constants.CollectorArmConstants.LIFT_MAX_HEIGHT)) {
            safePower = 0; // Stop movement
        }

        m_Lift.set(safePower);
    }

    // 🏗 New: Lift Increment Control (for button presses)
    public void incrementLift(boolean up) {
      final double targetHeight = MathUtil.clamp(
          getLiftHeightInches() + (up ? LIFT_INCREMENT : -LIFT_INCREMENT), 
          Constants.CollectorArmConstants.LIFT_MIN_HEIGHT, 
          Constants.CollectorArmConstants.LIFT_MAX_HEIGHT);
      
      double direction = (targetHeight > getLiftHeightInches()) ? 1.0 : -1.0;
      m_Lift.set(direction * 0.5); // Adjust speed as needed
  
      // Stop the motor when reaching target
      new Thread(() -> {
          while ((direction > 0 && getLiftHeightInches() < targetHeight) ||
                 (direction < 0 && getLiftHeightInches() > targetHeight)) {
              try {
                  Thread.sleep(10); // Small delay to check height periodically
              } catch (InterruptedException e) {
                  e.printStackTrace();
              }
          }
          m_Lift.set(0); // Stop motor
      }).start();

    }

    // 🏗 New: Joystick-Controlled Lift Movement with Deadband
    public Command liftCommand(DoubleSupplier joystickInput) {
        return new RunCommand(() -> {
            double input = -joystickInput.getAsDouble();
            if (Math.abs(input) > JOYSTICK_DEADBAND) {
                setLiftPower(input);
            } else {
                setLiftPower(0); // Stop if input is too small
            }
        }, this);
    }

    public Command StopLift() {
    return new InstantCommand(() -> m_Lift.stopMotor(), this);
}
      

    


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift Height", getLiftHeightInches());
    }
}
