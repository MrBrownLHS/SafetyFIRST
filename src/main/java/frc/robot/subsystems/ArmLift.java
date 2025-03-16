package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.ArmConfiguration;
import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class ArmLift extends SubsystemBase {
    private final SparkMax m_Lift;
    private final CANcoder liftEncoder;
    private final SparkMaxConfig motorConfig;
    

    private static final double LIFT_INCREMENT = 1.0;  // Inches per button press
    private static final double JOYSTICK_DEADBAND = 0.05; // Ignore small joystick movements

    public ArmLift() {
      m_Lift = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_ID, MotorType.kBrushless);
      liftEncoder = new CANcoder(Constants.CollectorArmConstants.LIFT_ENCODER_ID);
      motorConfig = new SparkMaxConfig();

      configureMotors(m_Lift, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
      configureEncoders(liftEncoder, Constants.CollectorArmConstants.ENCODER_TO_INCHES);
    }
   
    private void configureMotors(SparkMax motor, SparkMaxConfig config, int currentLimit) {
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(currentLimit);
        config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
        config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureEncoders(CANcoder encoder, double magnetOffset) {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
    
        double zeroOffset = encoder.getAbsolutePosition().getValueAsDouble(); // Read initial position
        magnetConfig.MagnetOffset = magnetOffset - zeroOffset; // Adjust offset dynamically
        magnetConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    
        encoderConfig.MagnetSensor = magnetConfig;
        encoder.getConfigurator().apply(encoderConfig);
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
