package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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



public class ArmLift extends SubsystemBase {
    private final SparkMax m_Lift;
    private final RelativeEncoder liftEncoder;
    private final SparkMaxConfig motorConfig;
    private final PIDController liftPID;
    private final TrapezoidProfile.Constraints liftConstraints;
    private TrapezoidProfile.State liftGoal, liftState;
    private final TrapezoidProfile liftProfile;
    private final ArmFeedforward liftFF;

    private static final double LIFT_COLLECT = 1.0;
    private static final double LIFT_L1 = 5.0;
    private static final double LIFT_L2 = 10.0;
    private static final double LIFT_L3 = 15.0;
    private static final double LIFT_FLOOR = 3.0;

   

    private static final double LIFT_MAX_VELOCITY = 10.0;
    private static final double LIFT_MAX_ACCELERATION = 20.0;

    private static final double LIFT_MAX_HEIGHT = 20.0;
    private static final double LIFT_MIN_HEIGHT = 0.0;

    private static final double LIFT_GEAR_RATIO = 64.0;
    private static final double COG_DIAMETER_INCHES = 2.0;
    private static final double LIFT_ENCODER_TO_INCHES = (Math.PI * COG_DIAMETER_INCHES) / LIFT_GEAR_RATIO;

    private static double kP = 0.1;
    private static double kI = 0.0;
    private static double kD = 0.01;
    private static final double kG = 0.4;

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

    public double getLiftHeight() {
        return liftEncoder.getPosition() * LIFT_ENCODER_TO_INCHES;
    }

    public void setPosition(double targetInches) {
        liftGoal = new TrapezoidProfile.State(
            MathUtil.clamp(targetInches, LIFT_MIN_HEIGHT, LIFT_MAX_HEIGHT), 0);   
        
    }

    public Command setLiftHeightCommand(double targetInches) {
        return new InstantCommand(() -> setPosition(targetInches), this)
        .andThen(new WaitUntilCommand(() -> Math.abs(getLiftHeight() - targetInches) < 0.5))
        .andThen(StopLift());
    }

    public Command moveLiftToL1() {
        return setLiftHeightCommand(LIFT_L1);
    }

    public Command moveLiftToL2() {
        return setLiftHeightCommand(LIFT_L2);
    }

    public Command moveLiftToL3() {
        return setLiftHeightCommand(LIFT_L3);
    }

    public Command moveLiftToCollect() {
        return setLiftHeightCommand(LIFT_COLLECT);
    }

    public Command moveLiftToFloor() {
        return setLiftHeightCommand(LIFT_FLOOR);
    }

    
    public Command StopLift() {
        return new InstantCommand(() -> {
            m_Lift.stopMotor();
            liftPID.reset();
            liftGoal = new TrapezoidProfile.State(getLiftHeight(), 0);
            liftState = new TrapezoidProfile.State(getLiftHeight(), 0);
            m_Lift.set(liftFF.calculate(0, getLiftHeight()));
        }, this);
    }
      
    @Override
    public void periodic() {
        double error = Math.abs(getLiftHeight() - liftGoal.position);
        if (error > 0.2) {
            liftState = liftProfile.calculate(0.02, liftState, liftGoal);
            double pidOutput = liftPID.calculate(getLiftHeight(), liftState.position);
            double feedforward = liftFF.calculate(0, liftGoal.position);
            double motorOutput = pidOutput + feedforward;
            m_Lift.set(MathUtil.clamp(motorOutput, -1.0, 1.0));
        } else {
            m_Lift.stopMotor();
        } 
              
        SmartDashboard.putNumber("Lift Height", getLiftHeight());

        if (SmartDashboard.getBoolean("Lift Tuning", false)) {
            kP = SmartDashboard.getNumber("Lift kP", kP);
            kI = SmartDashboard.getNumber("Lift kI", kI);
            kD = SmartDashboard.getNumber("Lift kD", kD);
            liftPID.setP(kP);
            liftPID.setI(kI);
            liftPID.setD(kD);
        }
    }
         
}

