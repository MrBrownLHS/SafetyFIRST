package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ArmPivot extends SubsystemBase {
    private final SparkMax m_Pivot;
    private final RelativeEncoder pivotEncoder;
    private final SparkMaxConfig motorConfig;
    private final PIDController pivotPID;
    private final TrapezoidProfile.Constraints pivotConstraints;
    private TrapezoidProfile.State pivotGoal, pivotState;
    private final TrapezoidProfile pivotProfile;
    private ArmFeedforward pivotFF;
    

    private static final double PIVOT_START = 0.0;
    private static final double PIVOT_COLLECT = -63.0;
    private static final double PIVOT_L1 = -117.0;
    private static final double PIVOT_L2 = -142;
    private static final double PIVOT_L3 = -297.0;
    private static final double PIVOT_MAX = -297.0;

    private static final double PIVOT_MAX_VELOCITY = 10.0; // Degrees per second
    private static final double PIVOT_MAX_ACCELERATION = 150.0; // Degrees per second²

    private static final double PIVOT_GEAR_RATIO = 84.0; //double check and update
    public static double PIVOT_ENCODER_TO_DEGREES = 360.0 / PIVOT_GEAR_RATIO;
    public static final double SOFT_STOP_BUFFER = 10.0;

    private static double kP = 0.05;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kG = 2.0;
    private static double kS = 0.01;
    private static double kV = 0.0;
    private static double kA = 0.0;

    public ArmPivot() {
        m_Pivot = new SparkMax(Constants.CollectorArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotEncoder = m_Pivot.getEncoder();
        motorConfig = new SparkMaxConfig();
        resetEncoder();

        pivotPID = new PIDController(kP, kI, kD);
        pivotPID.setTolerance(2.0);

        pivotFF = new ArmFeedforward(kS, kG, kV, kA);

        pivotConstraints = new TrapezoidProfile.Constraints(PIVOT_MAX_VELOCITY, PIVOT_MAX_ACCELERATION);
        pivotProfile = new TrapezoidProfile(pivotConstraints);

        pivotGoal = new TrapezoidProfile.State(PIVOT_START, 0);
        pivotState = new TrapezoidProfile.State(0.0, 0);

        SmartDashboard.putBoolean("Pivot Tuning", false);
        SmartDashboard.putNumber("Pivot kP", kP);
        SmartDashboard.putNumber("Pivot kI", kI);
        SmartDashboard.putNumber("Pivot kD", kD);
        SmartDashboard.putNumber("Pivot kS", kS);
        SmartDashboard.putNumber("Pivot kG", kG);
        SmartDashboard.putNumber("Pivot kV", kV);
        SmartDashboard.putNumber("Pivot kA", kA);
        
        

        configureMotors(m_Pivot, motorConfig, Constants.CollectorArmConstants.CURRENT_LIMIT_NEO);
        
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

    public double getPivotAngle() {
        return pivotEncoder.getPosition() *  PIVOT_ENCODER_TO_DEGREES;
    }

    public void setPivotPosition(double targetDegrees) {
        pivotGoal = new TrapezoidProfile.State(
            MathUtil.clamp(targetDegrees, PIVOT_START, PIVOT_MAX), 0);
        pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);
        double pidOutput = pivotPID.calculate(getPivotAngle(), pivotState.position);
        double feedforward = pivotFF.calculate(0, pivotState.position);
        double motorOutput = MathUtil.clamp(pidOutput + feedforward, -1.0, 1.0); 
        motorOutput = applySoftStop(motorOutput, getPivotAngle());  
        m_Pivot.set(motorOutput);
    }

    private double applySoftStop(double output, double armAngle) {
        if (armAngle < PIVOT_START + SOFT_STOP_BUFFER) {
            double scale = (armAngle - PIVOT_START) / SOFT_STOP_BUFFER;
            return output * MathUtil.clamp(armAngle, PIVOT_START, PIVOT_MAX); // Limit to min 20% speed
        } 
        if (armAngle > PIVOT_MAX - SOFT_STOP_BUFFER) {
            double scale = (PIVOT_MAX - armAngle) / SOFT_STOP_BUFFER;
            return output * MathUtil.clamp(armAngle, PIVOT_START, PIVOT_MAX); // Limit to min 20% speed
        }
        return output;
    }

    public boolean isAtTarget() {
        return pivotPID.atSetpoint();
    }

    public Command PivotToCollect() {
        return new InstantCommand(() -> setPivotPosition(PIVOT_COLLECT), this);
    }

    public Command PivotToL1() {
        return new InstantCommand(() -> setPivotPosition(PIVOT_L1), this);
    }

    public Command PivotToL2() {
        return new InstantCommand(() -> setPivotPosition(PIVOT_L2), this);
    }

    public Command PivotToL3() {
        return new InstantCommand(() -> setPivotPosition(PIVOT_L3), this);
    }  
    
    public RunCommand SimplePivotForward() {
        return new RunCommand(() -> {
            m_Pivot.set(0.5);
        }, this);
    }

    public RunCommand SimplePivotBack() {
        return new RunCommand(() -> {
            m_Pivot.set(-0.5);
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
      return new InstantCommand(() -> {
        m_Pivot.set(0.0);
        pivotPID.reset();
        pivotGoal = new TrapezoidProfile.State(getPivotAngle(), 0);
        pivotState = new TrapezoidProfile.State(getPivotAngle(), 0);
      }, this);
    }

    
    @Override
    public void periodic() {
        double error = Math.abs(getPivotAngle() - pivotGoal.position);
        if(error > 0.2) {
            pivotState = pivotProfile.calculate(0.02, pivotState, pivotGoal);
            double pidOutput = pivotPID.calculate(getPivotAngle(), pivotState.position);
            double feedforward = pivotFF.calculate(0, pivotGoal.position);
            double motorOutput = pidOutput + feedforward;
            motorOutput = applySoftStop(motorOutput, getPivotAngle());
            m_Pivot.set(MathUtil.clamp(motorOutput, -1.0, 1.0));
        } else {
            m_Pivot.stopMotor();
        }

        SmartDashboard.putNumber("Pivot Angle", getPivotAngle());

        if (SmartDashboard.getBoolean("Pivot Tuning", false)) {
            kP = SmartDashboard.getNumber("Pivot kP", kP);
            kI = SmartDashboard.getNumber("Pivot kI", kI);
            kD = SmartDashboard.getNumber("Pivot kD", kD);
            kS = SmartDashboard.getNumber("Pivot kS", kS);
            kG = SmartDashboard.getNumber("Pivot kG", kG);
            kV = SmartDashboard.getNumber("Pivot kV", kV);
            kA = SmartDashboard.getNumber("Pivot kA", kA);
            pivotPID.setP(kP);
            pivotPID.setI(kI);
            pivotPID.setD(kD);
            pivotFF = new ArmFeedforward(kS, kG, kV, kA);
        }
    }
}
