package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmPivot extends SubsystemBase {
    private PeriodicIO pivotPeriodicIO;
    private static ArmPivot pivotInstance;
    public static ArmPivot getInstance() {
        if (pivotInstance == null) {
            pivotInstance = new ArmPivot();
        }
        return pivotInstance;
    }

    private SparkMax m_PivotMotor;
    private RelativeEncoder pivotEncoder;
    private SparkClosedLoopController pivotPIDController;

    private TrapezoidProfile pivotProfile;
    private TrapezoidProfile.State pivotCurrentState = new TrapezoidProfile.State();
    private TrapezoidProfile.State pivotGoalState = new TrapezoidProfile.State();
    private double prevUpdateTime = Timer.getFPGATimestamp();

   


    private ArmPivot() {
        super("ArmPivot");

        
        pivotPeriodicIO = new PeriodicIO();

        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();

        pivotMotorConfig.closedLoop
            .pid(Constants.Pivot.PIVOT_kP, Constants.Pivot.PIVOT_kI, Constants.Pivot.PIVOT_kD)
            .iZone(Constants.Pivot.PIVOT_kIZone);
        
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO); 

        m_PivotMotor = new SparkMax(Constants.Pivot.PIVOT_MOTOR_ID, MotorType.kBrushless); 
        pivotEncoder = m_PivotMotor.getEncoder();
        pivotPIDController = m_PivotMotor.getClosedLoopController();
        m_PivotMotor.configure(
            pivotMotorConfig, 
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        pivotProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constants.Pivot.PIVOT_MAX_VELOCITY,
                Constants.Pivot.PIVOT_MAX_ACCELERATION
            )
        );
    }

    public enum PivotState {
        COLLECT,
        L1, 
        L2,
        L3,
        CLIMB
    }

    public static PivotState publicPivotState;

    public static class PeriodicIO {
        double pivot_target = 0.0;
        double pivot_power = 0.0;

        boolean is_pivot_positional_control = false;

        PivotState state = PivotState.COLLECT;
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Pivot Position", pivotEncoder.getPosition());
        SmartDashboard.putBoolean("Pivot Positional Control", pivotPeriodicIO.is_pivot_positional_control);
        SmartDashboard.putNumber("Pivot Target Position", pivotPeriodicIO.pivot_target);
        SmartDashboard.putString("Pivot State", pivotPeriodicIO.state.toString());

        double currentTime = Timer.getFPGATimestamp();
        double deltatime = currentTime - prevUpdateTime;
        prevUpdateTime = currentTime;

        if(pivotPeriodicIO.is_pivot_positional_control) {
            double currentPosition = pivotEncoder.getPosition();//added
            double error = Math.abs(currentPosition - pivotPeriodicIO.pivot_target);//added
            
            //pivotGoalState.position = pivotPeriodicIO.pivot_target;
            if (error > Constants.Pivot.PIVOT_POSITION_TOLERANCE) {

                pivotGoalState.position = pivotPeriodicIO.pivot_target;//added
                pivotCurrentState = pivotProfile.calculate(deltatime, pivotCurrentState, pivotGoalState);//added
            
            //prevUpdateTime = currentTime;
            //pivotCurrentState = pivotProfile.calculate(deltatime, pivotCurrentState, pivotGoalState);

                pivotPIDController.setReference(
                    pivotCurrentState.position, 
                    SparkBase.ControlType.kPosition,
                    ClosedLoopSlot.kSlot0,
                    Constants.Pivot.PIVOT_kG,
                    ArbFFUnits.kVoltage
                );
            }
        } else {
            pivotCurrentState.position = pivotEncoder.getPosition();
            pivotCurrentState.velocity = 0.0;
            m_PivotMotor.set(pivotPeriodicIO.pivot_power
            );
        }

        publicPivotState = pivotPeriodicIO.state;

    }

    public void writePeriodicOutputs() {

    }

    public void stopPivot() {
        pivotPeriodicIO.is_pivot_positional_control = false;
        pivotPeriodicIO.pivot_power = 0.0;

        m_PivotMotor.set(0.0);
    }

    public Command pivotReset() {
        return run(() -> pivotEncoder.setPosition(0.0));
    }

    public Command getPivotState() {
        return run(() -> getpivotstate());
    }

    private PivotState getpivotstate() {
        return pivotPeriodicIO.state;
    }

    public Command setPivotPower(double power) {
        return run(() -> setpivotpower(power));
    }

    private void setpivotpower(double power) {
        pivotPeriodicIO.is_pivot_positional_control = false;
        pivotPeriodicIO.pivot_power = power;
        m_PivotMotor.set(power);
    }

    private Command pivotToPosition(double targetPosition, PivotState state) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                pivotPeriodicIO.is_pivot_positional_control = true;
                pivotPeriodicIO.pivot_target = targetPosition;
                pivotPeriodicIO.state = state;
            }, this),
            new WaitUntilCommand(() -> 
                Math.abs(pivotEncoder.getPosition() - targetPosition) < Constants.Pivot.PIVOT_POSITION_TOLERANCE
            ).withTimeout(5.0),
            new InstantCommand(this::stopPivot, this)
        );
    }

    public Command pivotToCollect() {
        return pivotToPosition(Constants.Pivot.PIVOT_COLLECT_POS, PivotState.COLLECT);
    }
    
    public Command pivotToL1() {
        return pivotToPosition(Constants.Pivot.PIVOT_L1_POS, PivotState.L1);
    }

    public Command pivotToL2() {
        return pivotToPosition(Constants.Pivot.PIVOT_L2_POS, PivotState.L2);
    }

    public Command pivotToL3() {
        return pivotToPosition(Constants.Pivot.PIVOT_L3_POS, PivotState.L3);
    }

    public Command pivotToClimb() {
        return pivotToPosition(Constants.Pivot.PIVOT_CLIMB_POS, PivotState.CLIMB);
    }

    // public Command pivotToCollect() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = true;
    //             pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_COLLECT_POS;
    //             pivotPeriodicIO.state = PivotState.COLLECT;
    //         }, this),
    //         new WaitUntilCommand(() -> 
    //             Math.abs(pivotEncoder.getPosition() - Constants.Pivot.PIVOT_COLLECT_POS) < Constants.Pivot.PIVOT_POSITION_TOLERANCE
    //         ),
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = false;
    //             m_PivotMotor.set(0.0);
    //         }, this)
    //     );
    // }

    // public Command pivotToL1() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = true;
    //             pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_L1_POS;
    //             pivotPeriodicIO.state = PivotState.L1;
    //         }, this),
    //         new WaitUntilCommand(() -> 
    //             Math.abs(pivotEncoder.getPosition() - Constants.Pivot.PIVOT_L1_POS) < Constants.Pivot.PIVOT_POSITION_TOLERANCE
    //         ),
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = false;
    //             m_PivotMotor.set(0.0);
    //         }, this)
    //     );
    // }
    
    // public Command pivotToL2() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = true;
    //             pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_L2_POS;
    //             pivotPeriodicIO.state = PivotState.L2;
    //         }, this),
    //         new WaitUntilCommand(() -> 
    //             Math.abs(pivotEncoder.getPosition() - Constants.Pivot.PIVOT_L2_POS) < Constants.Pivot.PIVOT_POSITION_TOLERANCE
    //         ),
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = false;
    //             m_PivotMotor.set(0.0);
    //         }, this)
    //     );
    // }

    // public Command pivotToL3() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = true;
    //             pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_L3_POS;
    //             pivotPeriodicIO.state = PivotState.L3;
    //         }, this),
    //         new WaitUntilCommand(() -> 
    //             Math.abs(pivotEncoder.getPosition() - Constants.Pivot.PIVOT_L3_POS) < Constants.Pivot.PIVOT_POSITION_TOLERANCE
    //         ),
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = false;
    //             m_PivotMotor.set(0.0);
    //         }, this)
    //     );
    // }

    // public Command pivotToClimb() {
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = true;
    //             pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_CLIMB_POS;
    //             pivotPeriodicIO.state = PivotState.CLIMB;
    //         }, this),
    //         new WaitUntilCommand(() -> 
    //             Math.abs(pivotEncoder.getPosition() - Constants.Pivot.PIVOT_CLIMB_POS) < Constants.Pivot.PIVOT_POSITION_TOLERANCE
    //         ),
    //         new InstantCommand(() -> {
    //             pivotPeriodicIO.is_pivot_positional_control = false;
    //             m_PivotMotor.set(0.0);
    //         }, this)
    //     );
    // }

   


    // public Command pivotToCollect() {
    //     return run(() -> pivottocollect());
    // }

    // private void pivottocollect() {
    //     pivotPeriodicIO.is_pivot_positional_control = true;
    //     pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_COLLECT_POS;
    //     pivotPeriodicIO.state = PivotState.COLLECT;
    // }

    // public Command pivotToL1() {
    //     return run(() -> pivottol1());
    // }

    // private void pivottol1() {
    //     pivotPeriodicIO.is_pivot_positional_control = true;
    //     pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_L1_POS;
    //     pivotPeriodicIO.state = PivotState.L1;
    // }

    // public Command pivotToL2() {
    //     return run(() -> pivottol2());
    // }

    // private void pivottol2() {
    //     pivotPeriodicIO.is_pivot_positional_control = true;
    //     pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_L2_POS;
    //     pivotPeriodicIO.state = PivotState.L2;
    // }

    // public Command pivotToL3() {
    //     return run(() -> pivottol3());
    // }

    // private void pivottol3() {
    //     pivotPeriodicIO.is_pivot_positional_control = true;
    //     pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_L3_POS;
    //     pivotPeriodicIO.state = PivotState.L3;
    // }

    // public Command pivotToClimb() {
    //     return run(() -> pivottoclimb());
    // }

    // private void pivottoclimb() {
    //     pivotPeriodicIO.is_pivot_positional_control = true;
    //     pivotPeriodicIO.pivot_target = Constants.Pivot.PIVOT_CLIMB_POS;
    //     pivotPeriodicIO.state = PivotState.CLIMB;
    // }

}

