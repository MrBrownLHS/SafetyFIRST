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
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;


//https://github.com/Little-Apple-Cyborgs-5968/FRC2025/blob/main/src/main/java/frc/robot/subsystems/Elevator.java

public class ArmLift extends SubsystemBase {
    private PeriodicIO liftPeriodicIO;
    private static ArmLift liftInstance;
    public static ArmLift getInstance() {
        if (liftInstance == null) {
            liftInstance = new ArmLift();
        }
        return liftInstance;
    }

    private SparkMax m_LiftMotor;
    private RelativeEncoder liftEncoder;.
    private SparkClosedLoopController liftPIDController;

    private TrapezoidProfile liftProfile;
    private TrapezoidProfile.State liftCurrentState = new TrapezoidProfile.State();
    private TrapezoidProfile.State liftGoalState = new TrapezoidProfile.State();
    private double prevUpdateTime = Timer.getFPGATimestamp();

    private static final double LIFT_MAX_VELOCITY = 5.0;
    private static final double LIFT_MAX_ACCELERATION = 20.0;

    // private static final double LIFT_MAX_HEIGHT = -18.0;
    // private static final double LIFT_MIN_HEIGHT = 0.0;

    // private static final double LIFT_GEAR_RATIO = 64.0;
    // private static final double COG_DIAMETER_INCHES = 2.0;
    // private static final double LIFT_ENCODER_TO_INCHES = (Math.PI * COG_DIAMETER_INCHES) / LIFT_GEAR_RATIO;

    // private static double kP = 0.1;
    // private static double kI = 0.0;
    // private static double kD = 0.0;
  
    private ArmLift() {
        super("ArmLift");

        liftPeriodicIO = new PeriodicIO();
        SparkMaxConfig liftMotorConfig = new SparkMaxConfig();

        liftMotorConfig.getClosedLoopController
        .pid(Constants.Lift.LIFT_kP, Constants.Lift.LIFT_kI, Constants.Lift.LIFT_kD) //Add these constants to Constants
        .iZone(Constants.Lift.Lift_kIZone);

        liftMotorConfig.idleMode(IdleMode.kBrake);
        liftMotorConfig.smartCurrentLimit(Constants.ConfigureConstants.CURRENT_LIMIT_NEO); //Reorganize Constants
       
        m_LiftMotor = new SparkMax(Constants.Lift.LIFT_MOTOR_ID, MotorType.kBrushless); //Reorganize Constants
        liftEncoder = m_LiftMotor.getEncoder();
        liftPIDController = m_LiftMotor.getClosedLoopController();
        m_LiftMotor.configure(
            liftMotorConfig, 
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        liftProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(LIFT_MAX_VELOCITY, LIFT_MAX_ACCELERATION)
        );
    }

    public enum LiftState {
        START,
        COLLECT,
        L1,
        L2,
        L3
    }

    public static LiftState publicLiftState;

    public static class PeriodicIO {
        double lift_target;
        double lift_power;

        double is_lift_positional_control = false;
        LiftState state = LiftState.START;
    }

    @Override

    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - prevUpdateTime;
        prevUpdateTime = currentTime;
        if (liftPeriodicIO.is_lift_positional_control) {
            liftGoalState.position = liftPeriodicIO.lift_target;
            prevUpdateTime = currentTime;
            liftCurrentState = liftProfile.calculate(deltaTime, liftCurrentState, liftGoalState);

            liftPIDController.setReference(
                liftCurrentState.position,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.liftSlot0,
                Constants.Lift.LIFT_kG,
                ArbFFUnits.kVoltage);
        } else {
            liftCurrentState.position = liftEncoder.getPosition();
            liftCurrentState.velocity = 0.0;
            liftMotor.set(liftPeriodicIO.lift_power
            );
        }

        publicLiftState = liftPeriodicIO.state;

        }

    public void writePeriodicOutputs() {

    }

    public void stopLift() {
        liftPeriodicIO.is_lift_positional_control = false;
        liftPeriodicIO.lift_power = 0.0;
        
        m_LiftMotor.set(0.0);
    }

    public Command liftReset() {
        return run(() -> getstate());
    }

    private LiftState getstate() {
        return liftPeriodicIO.state;
    }

    public Command setLiftPower(double power) {
        liftPeriodicIO.is_lift_positional_control = false;
        liftPeriodicIO.lift_power = power;
    }

    private void liftToStart() {
        return run(() -> lifttostart());
    }

    private void lifttostart() {
        liftPeriodicIO.is_lift_positional_control = true;
        liftPeriodicIO.lift_target = Constants.Lift.LIFT_START_POS;
        liftPeriodicIO.state = LiftState.START;
    }



         
                      
        
}

