package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmPivot extends SubsystemBase {
    private final SparkMax m_Pivot
    = new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);;
    private final RelativeEncoder pivotEncoder 
    = m_Pivot.getEncoder();
    private final SparkMaxConfig motorConfig 
    = new SparkMaxConfig();
    private final PIDController pivotPID 
    = new PIDController(kP, kI, kD);
    private final TrapezoidProfile.Constraints pivotConstraints 
    = new TrapezoidProfile.Constraints(PIVOT_MAX_VELOCITY, PIVOT_MAX_ACCELERATION);
    private TrapezoidProfile.State pivotGoal 
    = new TrapezoidProfile.State(PIVOT_START, 0);
    private TrapezoidProfile.State pivotState 
    = new TrapezoidProfile.State(0.0, 0);;
    private final TrapezoidProfile pivotProfile
    = new TrapezoidProfile(pivotConstraints);


    private static final double PIVOT_START = 0.0;
    private static final double PIVOT_COLLECT = -63.0;
    private static final double PIVOT_L1 = -117.0;
    private static final double PIVOT_L2 = -142;
    private static final double PIVOT_L3 = -297.0;
    private static final double PIVOT_MAX = -297.0;

    private static final double PIVOT_MAX_VELOCITY = 10.0; // Degrees per second
    private static final double PIVOT_MAX_ACCELERATION = 150.0; // Degrees per second²

    private static final double PIVOT_GEAR_RATIO = 100.0; //double check and update
    public static double PIVOT_ENCODER_TO_DEGREES = 360.0 / PIVOT_GEAR_RATIO;
    
    private static double kP = 0.05;
    private static double kI = 0.0;
    private static double kD = 0.0;

    public ArmPivot() {
        configureMotors(m_Pivot, motorConfig, Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
        resetEncoder();
        configurePID();
        updateSmartDashboard();
    }
        
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Pivot Angle", getPivotAngle());
        SmartDashboard.putBoolean("Pivot Tuning", false);
        SmartDashboard.putNumber("Pivot kP", SmartDashboard.getNumber("Pivot kP", kP));
        SmartDashboard.putNumber("Pivot kI", SmartDashboard.getNumber("Pivot kI", kI));
        SmartDashboard.putNumber("Pivot kD", SmartDashboard.getNumber("Pivot kD", kD));    
    }

    private void configureMotors(SparkMax motor, SparkMaxConfig config, int currentLimit) {
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(currentLimit);
      config.secondaryCurrentLimit(Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
      config.voltageCompensation(Constants.CollectorArmConstants.VOLTAGE_COMPENSATION);
      motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void resetEncoder() {
        pivotEncoder.setPosition(0.0);
    }

    private void configurePID() {
        pivotPID.setP(kP);
        pivotPID.setI(kI);
        pivotPID.setD(kD);
        pivotPID.setTolerance(0.1);
    }

    private double encoderToDegrees(double encoderPosition) {
        return encoderPosition * PIVOT_ENCODER_TO_DEGREES;
    }
    
    private double degreesToEncoder(double degrees) {
        return degrees / PIVOT_ENCODER_TO_DEGREES;
    }
     
    public double getPivotAngle() {
        return encoderToDegrees(pivotEncoder.getPosition());
    }

    public void setPivotPosition(double targetDegrees) {
        pivotGoal = new TrapezoidProfile.State(
            MathUtil.clamp(targetDegrees, PIVOT_START, PIVOT_MAX), 0);
        pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);
        double positionRotations = degreesToEncoder(pivotGoal.position);
        m_Pivot.getClosedLoopController().setReference(positionRotations, ControlType.kPosition);
    }

    
    public boolean isAtTarget() {
        return pivotPID.atSetpoint();
    }

    public Command pivotToPosition(double targetDegrees) {
        return new InstantCommand(() -> setPivotPosition(targetDegrees), this);
    }

    public Command PivotToCollect() {
        return pivotToPosition(PIVOT_COLLECT); 
    }

    public Command PivotToL1() {
        return pivotToPosition(PIVOT_L1);
    }

    public Command PivotToL2() {
        return pivotToPosition(PIVOT_L2);
    }

    public Command PivotToL3() {
        return pivotToPosition(PIVOT_L3);
    }  
    
    public RunCommand SimplePivotForward() {
        return new RunCommand(() -> {
            m_Pivot.set(-0.5);
        }, this);
    }

    public RunCommand SimplePivotBack() {
        return new RunCommand(() -> {
            m_Pivot.set(0.5);
        }, this);
    }

    public RunCommand setPivotPositionManual(double targetDegrees) {
        return new RunCommand(() -> setPivotPosition(targetDegrees), this);
    }

    public RunCommand ManualPivotToMax() {
        return setPivotPositionManual(PIVOT_MAX);
    }

    public RunCommand ManualPivotToMin() {
        return setPivotPositionManual(PIVOT_START);
    }


    

    public Command StopPivot() {
        return new InstantCommand(() -> m_Pivot.set(0.0), this);
    }

    
    @Override
    public void periodic() {
        updateSmartDashboard();
    SmartDashboard.putNumber("Pivot Angle", getPivotAngle());

        if (SmartDashboard.getBoolean("Pivot Tuning", false)) {
            double newKP = SmartDashboard.getNumber("Pivot kP", kP);
            double newKI = SmartDashboard.getNumber("Pivot kI", kI);
            double newKD = SmartDashboard.getNumber("Pivot kD", kD);

            if (newKP != kP || newKI != kI || newKD != kD) {
                kP = newKP;
                kI = newKI;
                kD = newKD;
                configurePID();
                System.out.println("Updated PID values: kP=" + kP + ", kI=" + kI + ", kD=" + kD);
            }
        }
    }
}
