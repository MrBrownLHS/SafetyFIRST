package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.constants.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;



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
    private RelativeEncoder liftEncoder;
    private SparkClosedLoopController liftPIDController;

    private TrapezoidProfile liftProfile;
    private TrapezoidProfile.State liftCurrentState = new TrapezoidProfile.State();
    private TrapezoidProfile.State liftGoalState = new TrapezoidProfile.State();
    private double prevUpdateTime = Timer.getFPGATimestamp();

     
    private ArmLift() {
        super("ArmLift");

        liftPeriodicIO = new PeriodicIO();

        SparkMaxConfig liftMotorConfig = new SparkMaxConfig();

        liftMotorConfig.closedLoop
            .pid(Constants.Lift.LIFT_kP, Constants.Lift.LIFT_kI, Constants.Lift.LIFT_kD)
            .iZone(Constants.Lift.LIFT_kIZone);

        liftMotorConfig.idleMode(IdleMode.kBrake);
        liftMotorConfig.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO); //Reorganize Constants
       
        m_LiftMotor = new SparkMax(Constants.Lift.LIFT_MOTOR_ID, MotorType.kBrushless); //Reorganize Constants
        liftEncoder = m_LiftMotor.getEncoder();
        liftPIDController = m_LiftMotor.getClosedLoopController();
        m_LiftMotor.configure(
            liftMotorConfig, 
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        liftProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constants.Lift.LIFT_MAX_VELOCITY, 
                Constants.Lift.LIFT_MAX_ACCELERATION
                )
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
        double lift_target = 0.0;
        double lift_power = 0.0;

        boolean is_lift_positional_control = false;
        
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
               ClosedLoopSlot.kSlot0,
                Constants.Lift.LIFT_kG,
                ArbFFUnits.kVoltage);
        } else {
            liftCurrentState.position = liftEncoder.getPosition();
            liftCurrentState.velocity = 0.0;
            m_LiftMotor.set(liftPeriodicIO.lift_power);
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
        return run(() -> liftEncoder.setPosition(0.0));
    }

    public Command getLiftState() {
        return run(() -> getliftstate());
    }

    private LiftState getliftstate() {
        return liftPeriodicIO.state;
    }

    public Command setLiftPower(double power) {
        return run(() -> setliftpower(power));
    }

    private void setliftpower(double power) {
        liftPeriodicIO.is_lift_positional_control = false;
        liftPeriodicIO.lift_power = power;
    }

    public Command liftToStart() {
        return run(() -> lifttostart());
    }

    private void lifttostart() {
        liftPeriodicIO.is_lift_positional_control = true;
        liftPeriodicIO.lift_target = Constants.Lift.LIFT_START_POS;
        liftPeriodicIO.state = LiftState.START;
    }

    public Command liftToCollect() {
        return run(() -> lifttocollect());
    }

    private void lifttocollect() {
        liftPeriodicIO.is_lift_positional_control = true;
        liftPeriodicIO.lift_target = Constants.Lift.LIFT_COLLECT_POS;
        liftPeriodicIO.state = LiftState.COLLECT;
    }

    public Command liftToL1() {
        return run(() -> lifttol1());
    }

    private void lifttol1() {
        liftPeriodicIO.is_lift_positional_control = true;
        liftPeriodicIO.lift_target = Constants.Lift.LIFT_L1_POS;
        liftPeriodicIO.state = LiftState.L1;
    }

    public Command liftToL2() {
        return run(() -> lifttol2());
    }

    private void lifttol2() {
        liftPeriodicIO.is_lift_positional_control = true;
        liftPeriodicIO.lift_target = Constants.Lift.LIFT_L2_POS;
        liftPeriodicIO.state = LiftState.L2;
    }

    public Command liftToL3() {
        return run(() -> lifttol3());
    }

    private void lifttol3() {
        liftPeriodicIO.is_lift_positional_control = true;
        liftPeriodicIO.lift_target = Constants.Lift.LIFT_L3_POS;
        liftPeriodicIO.state = LiftState.L3;
    }




         
                      
        
}

