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

import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmPivot extends SubsystemBase {
    private final SparkMax m_Pivot;
    private final CANcoder pivotEncoder;
    private final SparkMaxConfig motorConfig;

    private static final double PIVOT_INCREMENT = 1.0;  // Inches per button press
    private static final double JOYSTICK_DEADBAND = 0.05; // Ignore small joystick movements

    public ArmPivot() {
        m_Pivot = new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = new CANcoder(Constants.CollectorArmConstants.PIVOT_ENCODER_ID);
        motorConfig = new SparkMaxConfig();

        configureMotors(m_Pivot, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
        configureEncoders(pivotEncoder, Constants.CollectorArmConstants.ENCODER_TO_INCHES);
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


    public double getPivotAngleDegrees() {
      return pivotEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_DEGREES;
    }

    public void setPivotPower(double power) {
        double currentAngle = getPivotAngleDegrees();
        double safePower = MathUtil.clamp(power, -1.0, 1.0);

      
        if ((safePower < 0 && currentAngle <= Constants.CollectorArmConstants.PIVOT_MIN_ANGLE) || 
            (safePower > 0 && currentAngle >= Constants.CollectorArmConstants.PIVOT_MAX_ANGLE)) {
            safePower = 0; // Stop movement
        }

        m_Pivot.set(safePower);
    }

    
    public void incrementPivot(boolean up) {
        final double targetAngle = MathUtil.clamp(
          getPivotAngleDegrees() + (up ? PIVOT_INCREMENT : -PIVOT_INCREMENT),
          Constants.CollectorArmConstants.PIVOT_MIN_ANGLE, 
          Constants.CollectorArmConstants.PIVOT_MAX_ANGLE);

         double direction = (targetAngle > getPivotAngleDegrees()) ? 1.0 : -1.0;
         m_Pivot.set(direction * 0.5); // Adjust speed as needed


         new Thread(() -> {
          while ((direction > 0 && getPivotAngleDegrees() < targetAngle) ||
                 (direction < 0 && getPivotAngleDegrees() > targetAngle)) {
              try {
                  Thread.sleep(10); // Small delay to check height periodically
              } catch (InterruptedException e) {
                  e.printStackTrace();
              }
          }
          m_Pivot.set(0); // Stop motor
      }).start();
        }


    
    public Command pivotCommand(DoubleSupplier joystickInput) {
        return new RunCommand(() -> {
            double input = -joystickInput.getAsDouble();
            if (Math.abs(input) > JOYSTICK_DEADBAND) {
                setPivotPower(input);
            } else {
                setPivotPower(0); // Stop if input is too small
            }
        }, this);
    }

    public Command StopPivot() {
      return new InstantCommand(() -> m_Pivot.stopMotor(), this);
      }

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber(":Pivot Angle", getPivotAngleDegrees());
    }
}
