package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;
import java.util.jar.Attributes.Name;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.ArmConfiguration;
import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.math.MathUtil;

public class ArmLift extends SubsystemBase {
    private final ArmConfiguration armConfig;
    private boolean tuningMode = false;

    private static final double LIFT_INCREMENT = 1.0;  // Inches per button press
    private static final double JOYSTICK_DEADBAND = 0.05; // Ignore small joystick movements

    public ArmLift() {
        armConfig = new ArmConfiguration();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.log("ArmLift Subsystem Initialized");
        SmartDashboard.putBoolean("Tuning Mode", false);
    }

    public double getLiftHeightInches() {
        return armConfig.liftEncoder.getAbsolutePosition().getValueAsDouble() * Constants.CollectorArmConstants.ENCODER_TO_INCHES;
    }

    public void setLiftPower(double power) {
        double currentHeight = getLiftHeightInches();
        double safePower = MathUtil.clamp(power, -1.0, 1.0);

        // Prevent movement beyond limits
        if ((safePower < 0 && currentHeight <= Constants.CollectorArmConstants.LIFT_MIN_HEIGHT) || 
            (safePower > 0 && currentHeight >= Constants.CollectorArmConstants.LIFT_MAX_HEIGHT)) {
            safePower = 0; // Stop movement
        }

        armConfig.m_Lift.set(safePower);
    }

    // 🏗 New: Lift Increment Control (for button presses)
    public void incrementLift(boolean up) {
        double targetHeight = getLiftHeightInches() + (up ? LIFT_INCREMENT : -LIFT_INCREMENT);

        // Clamp the height within safe limits
        targetHeight = MathUtil.clamp(targetHeight, 
                                      Constants.CollectorArmConstants.LIFT_MIN_HEIGHT, 
                                      Constants.CollectorArmConstants.LIFT_MAX_HEIGHT);

        // Move the lift motor to the new target position
        double direction = (targetHeight > getLiftHeightInches()) ? 1.0 : -1.0;
        armConfig.m_Lift.set(direction * 0.5); // Adjust speed as needed
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

    public void stopLift() {
        armConfig.m_Lift.stopMotor();
    }

    public void setTuningMode(boolean enabled) {
        tuningMode = enabled;
        SmartDashboard.putBoolean("Tuning Mode", tuningMode);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift Height", getLiftHeightInches());
    }
}
