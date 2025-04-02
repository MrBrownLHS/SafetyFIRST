package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;




public class ArmLift extends SubsystemBase {
    private final SparkMax m_Lift 
    = new SparkMax(Constants.CollectorArmConstants.LIFT_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder liftEncoder
    = m_Lift.getEncoder();
    private final SparkMaxConfig motorConfig 
    = new SparkMaxConfig();
    private final PIDController liftPID 
    = new PIDController(kP, kI, kD);
    private final TrapezoidProfile.Constraints liftConstraints 
    = new TrapezoidProfile.Constraints(LIFT_MAX_VELOCITY, LIFT_MAX_ACCELERATION);
    private TrapezoidProfile.State liftGoal 
    = new TrapezoidProfile.State(LIFT_MAX_HEIGHT, 0);
    private TrapezoidProfile.State liftState 
    = new TrapezoidProfile.State(0.0, 0);
    private final TrapezoidProfile liftProfile 
    = new TrapezoidProfile(liftConstraints);
 
    private static final double LIFT_COLLECT = -5.0;
    private static final double LIFT_L1 = -10.0;
    private static final double LIFT_L2 = -15.0;
    private static final double LIFT_L3 = -18.0;
    

   

    private static final double LIFT_MAX_VELOCITY = 5.0;
    private static final double LIFT_MAX_ACCELERATION = 20.0;

    private static final double LIFT_MAX_HEIGHT = -18.0;
    private static final double LIFT_MIN_HEIGHT = -1.0;

    private static final double LIFT_GEAR_RATIO = 64.0;
    private static final double COG_DIAMETER_INCHES = 2.0;
    private static final double LIFT_ENCODER_TO_INCHES = (Math.PI * COG_DIAMETER_INCHES) / LIFT_GEAR_RATIO;

    private static double kP = 0.025;
    private static double kI = 0.0;
    private static double kD = 0.0;
  
    public ArmLift() {
      configureMotors(m_Lift, motorConfig, Constants.CollectorArmConstants.MAX_CURRENT_LIMIT_NEO);
      resetEncoder();
      configurePID();
      updateSmartDashboard();
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Lift Height", getLiftHeight());
        SmartDashboard.putBoolean("Lift Tuning", false);
        SmartDashboard.putNumber("Lift kP", SmartDashboard.getNumber("Lift kP", kP));
        SmartDashboard.putNumber("Lift kI", SmartDashboard.getNumber("Lift kI", kI));
        SmartDashboard.putNumber("Lift kD", SmartDashboard.getNumber("Lift kD", kD));    
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

    private void configurePID() {
        liftPID.setP(kP);
        liftPID.setI(kI);
        liftPID.setD(kD);
    }

    private double encoderToInches(double encoderValue) {
        return encoderValue * LIFT_ENCODER_TO_INCHES;
    }

    private double inchesToEncoder(double inches) {
        return inches / LIFT_ENCODER_TO_INCHES;
    }

    public double getLiftHeight() {
        return encoderToInches(liftEncoder.getPosition());
    }

    public void setLiftPosition(double targetInches) {
        liftGoal = new TrapezoidProfile.State(
            MathUtil.clamp(targetInches, LIFT_MIN_HEIGHT, LIFT_MAX_HEIGHT), 0);
        liftState = liftProfile.calculate(0.02, liftState, liftGoal);
        double distanceRotations = inchesToEncoder(liftGoal.position);
        m_Lift.getClosedLoopController().setReference(distanceRotations, ControlType.kPosition);  
    }

    public boolean isLiftAtTarget() {
        return liftPID.atSetpoint();
    }

    public Command setLiftHeight(double targetInches) {
        return new InstantCommand(() -> setLiftPosition(targetInches), this);
    }

    public Command LiftToCollect() {
        return setLiftHeight(LIFT_COLLECT);
    }

    public Command LiftToL1() {
        return setLiftHeight(LIFT_L1);
    }

    public Command LiftToL2() {
        return setLiftHeight(LIFT_L2);
    }

    public Command LiftToL3() {
        return setLiftHeight(LIFT_L3);
    }

    public RunCommand SimpleLiftUp() {
        return new RunCommand(() -> {
            m_Lift.set(-0.5);
        }, this);
    }

    public RunCommand SimpleLiftDown() {
        return new RunCommand(() -> {
            m_Lift.set(0.5);
        }, this);
    }

    public Command StopLift() {
        return new InstantCommand(() -> m_Lift.set(0.0), this);
    }
      
    @Override

    public void periodic() {
        updateSmartDashboard();
                      
        SmartDashboard.putNumber("Lift Height", getLiftHeight());

        if (SmartDashboard.getBoolean("Lift Tuning", false)) {
            double newKP = SmartDashboard.getNumber("Lift kP", kP);
            double newKI = SmartDashboard.getNumber("Lift kI", kI);
            double newKD = SmartDashboard.getNumber("Lift kD", kD);
           
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

