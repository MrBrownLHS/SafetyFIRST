// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.constants.Constants;
//import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmRotate extends SubsystemBase {
  private PeriodicIO rotatePeriodicIO;
  private static ArmRotate rotateInstance;
  public static ArmRotate getInstance() {
    if (rotateInstance == null) {
      rotateInstance = new ArmRotate();
    }
    return rotateInstance;

  }
  private SparkMax m_RotateMotor;
  private RelativeEncoder rotateEncoder;
  private SparkClosedLoopController rotatePIDController;

  private TrapezoidProfile rotateProfile;
  private TrapezoidProfile.State rotateCurrentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State rotateGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();
  
 

  private ArmRotate() {
    super("ArmRotate");

    
    rotatePeriodicIO = new PeriodicIO();

    SparkMaxConfig rotateMotorConfig = new SparkMaxConfig();

    rotateMotorConfig.closedLoop
        .pid(Constants.Rotate.ROTATE_kP, Constants.Rotate.ROTATE_kI, Constants.Rotate.ROTATE_kD)
        .iZone(Constants.Rotate.ROTATE_kIZone);

    rotateMotorConfig.idleMode(IdleMode.kBrake);
    rotateMotorConfig.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);

    m_RotateMotor = new SparkMax(Constants.Rotate.ROTATE_MOTOR_ID, MotorType.kBrushless);
    rotateEncoder = m_RotateMotor.getEncoder();
    rotatePIDController = m_RotateMotor.getClosedLoopController();
    m_RotateMotor.configure(
        rotateMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    rotateProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            Constants.Rotate.ROTATE_MAX_VELOCITY,
            Constants.Rotate.ROTATE_MAX_ACCELERATION
        )
    );
  }

  public enum RotateState {
    COLLECT,
    L1,
    L2,
    L3,
    CLIMB
  }

  public static RotateState publicRotateState;

  public static class PeriodicIO {
    double rotate_target = 0.0;
    double rotate_power = 0.0;

    boolean is_rotate_positional_control = false;

    RotateState state = RotateState.COLLECT;
  }
 

  @Override
    public void periodic() {
      SmartDashboard.putNumber("Arm Rotate Position", rotateEncoder.getPosition());
      SmartDashboard.putBoolean("Rotate Positional Control", rotatePeriodicIO.is_rotate_positional_control);
      SmartDashboard.putNumber("Rotate Target Position", rotatePeriodicIO.rotate_target);
      SmartDashboard.putString("Rotate State", rotatePeriodicIO.state.toString());

      double currentTime = Timer.getFPGATimestamp();
      double deltaTime = currentTime - prevUpdateTime;
      prevUpdateTime = currentTime;
      if(rotatePeriodicIO.is_rotate_positional_control) {
          rotateGoalState.position = rotatePeriodicIO.rotate_target;

          prevUpdateTime = currentTime;
          rotateCurrentState = rotateProfile.calculate(deltaTime, rotateCurrentState, rotateGoalState);

          rotatePIDController.setReference(
              rotateCurrentState.position, 
              SparkBase.ControlType.kPosition,
              ClosedLoopSlot.kSlot0,
              Constants.Rotate.ROTATE_kG,
              ArbFFUnits.kVoltage);
      } else {
          rotateCurrentState.position = rotateEncoder.getPosition();
          rotateCurrentState.velocity = 0.0;
          m_RotateMotor.set(rotatePeriodicIO.rotate_power
          );
      }
      publicRotateState = rotatePeriodicIO.state; // Update publicRotateState with the current state
      publicRotateState = rotatePeriodicIO.state;
      
    }

    public void writePeriodicOutputs() {

    }

    public void stopRotate() {
      rotatePeriodicIO.is_rotate_positional_control = false;
      rotatePeriodicIO.rotate_power = 0.0;

      m_RotateMotor.set(0.0);
    }

    public Command rotateReset() {
      return run (() -> rotateEncoder.setPosition(0.0));
    }

    public Command getRotateState() {
      return run(() -> getrotatestate());
    }

    private RotateState getrotatestate() {
      return rotatePeriodicIO.state;
    }

    public Command setRotatePower(double power) {
      return run (() -> setrotatepower(power));
    }

    private void setrotatepower(double power) {
      rotatePeriodicIO.is_rotate_positional_control = false;
      rotatePeriodicIO.rotate_power = power;
      m_RotateMotor.set(power);
    }

    private Command rotateToPosition(double tartgetPosition, RotateState state) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> {
              rotatePeriodicIO.is_rotate_positional_control = true;
              rotatePeriodicIO.rotate_target = tartgetPosition;
              rotatePeriodicIO.state = state;
          }, this),
          new WaitUntilCommand(() -> 
              Math.abs(rotateEncoder.getPosition() - tartgetPosition) < Constants.Rotate.ROTATE_POSITION_TOLERANCE
          ).withTimeout(5.0),
          new InstantCommand(this::stopRotate, this)
      );
    }

    public Command rotateToCollect() {
      return rotateToPosition(Constants.Rotate.ROTATE_COLLECT_POS, RotateState.COLLECT);
    }

    public Command rotateToL1() {
      return rotateToPosition(Constants.Rotate.ROTATE_L1_POS, RotateState.L1);
    }

    public Command rotateToL2() {
      return rotateToPosition(Constants.Rotate.ROTATE_L2_POS, RotateState.L2);
    }

    public Command rotateToL3() {
      return rotateToPosition(Constants.Rotate.ROTATE_L3_POS, RotateState.L3);
    }

    public Command rotateToClimb() {
      return rotateToPosition(Constants.Rotate.ROTATE_CLIMB_POS, RotateState.CLIMB);
    }

  //   public Command rotateToCollect() {
  //       return new SequentialCommandGroup(
  //           new InstantCommand(() -> {
  //               rotatePeriodicIO.is_rotate_positional_control = true;
  //               rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_COLLECT_POS;
  //               rotatePeriodicIO.state = RotateState.COLLECT;
  //           }, this),
  //           new WaitUntilCommand(() -> 
  //               Math.abs(rotateEncoder.getPosition() - Constants.Rotate.ROTATE_COLLECT_POS) < Constants.Rotate.ROTATE_POSITION_TOLERANCE
  //           ),
  //           new InstantCommand(() -> {
  //               rotatePeriodicIO.is_rotate_positional_control = false;
  //               m_RotateMotor.set(0.0);
  //           }, this)
  //       );
  //   }

  //   public Command rotateToL1() {
  //     return new SequentialCommandGroup(
  //         new InstantCommand(() -> {
  //             rotatePeriodicIO.is_rotate_positional_control = true;
  //             rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_L1_POS;
  //             rotatePeriodicIO.state = RotateState.L1;
  //         }, this),
  //         new WaitUntilCommand(() -> 
  //             Math.abs(rotateEncoder.getPosition() - Constants.Rotate.ROTATE_L1_POS) < Constants.Rotate.ROTATE_POSITION_TOLERANCE
  //         ),
  //         new InstantCommand(() -> {
  //             rotatePeriodicIO.is_rotate_positional_control = false;
  //             m_RotateMotor.set(0.0);
  //         }, this)
  //     );
  //   }

  //   public Command rotateToL2() {
  //     return new SequentialCommandGroup(
  //         new InstantCommand(() -> {
  //             rotatePeriodicIO.is_rotate_positional_control = true;
  //             rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_L2_POS;
  //             rotatePeriodicIO.state = RotateState.L2;
  //         }, this),
  //         new WaitUntilCommand(() -> 
  //             Math.abs(rotateEncoder.getPosition() - Constants.Rotate.ROTATE_L2_POS) < Constants.Rotate.ROTATE_POSITION_TOLERANCE
  //         ),
  //         new InstantCommand(() -> {
  //             rotatePeriodicIO.is_rotate_positional_control = false;
  //             m_RotateMotor.set(0.0);
  //         }, this)
  //     );
  //   }

  //   public Command rotateToL3() {
  //     return new SequentialCommandGroup(
  //         new InstantCommand(() -> {
  //             rotatePeriodicIO.is_rotate_positional_control = true;
  //             rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_L3_POS;
  //             rotatePeriodicIO.state = RotateState.L3;
  //         }, this),
  //         new WaitUntilCommand(() -> 
  //             Math.abs(rotateEncoder.getPosition() - Constants.Rotate.ROTATE_L3_POS) < Constants.Rotate.ROTATE_POSITION_TOLERANCE
  //         ),
  //         new InstantCommand(() -> {
  //             rotatePeriodicIO.is_rotate_positional_control = false;
  //             m_RotateMotor.set(0.0);
  //         }, this)
  //     );
  //   }

  //   public Command rotateToClimb() {
  //     return new SequentialCommandGroup(
  //         new InstantCommand(() -> {
  //             rotatePeriodicIO.is_rotate_positional_control = true;
  //             rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_CLIMB_POS;
  //             rotatePeriodicIO.state = RotateState.CLIMB;
  //         }, this),
  //         new WaitUntilCommand(() -> 
  //             Math.abs(rotateEncoder.getPosition() - Constants.Rotate.ROTATE_CLIMB_POS) < Constants.Rotate.ROTATE_POSITION_TOLERANCE
  //         ),
  //         new InstantCommand(() -> {
  //             rotatePeriodicIO.is_rotate_positional_control = false;
  //             m_RotateMotor.set(0.0);
  //         }, this)
  //     );
  // }




  // public Command rotateToCollect() {
  //   return run(() -> rotatetocollect());
  // }

  // private void rotatetocollect() {
  //   rotatePeriodicIO.is_rotate_positional_control = true;
  //   rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_COLLECT_POS;
  //   rotatePeriodicIO.state = RotateState.COLLECT;
  // }

  // public Command rotateToL1() {
  //   return run(() -> rotatetol1());
  // }

  // private void rotatetol1() {
  //   rotatePeriodicIO.is_rotate_positional_control = true;
  //   rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_L1_POS;
  //   rotatePeriodicIO.state = RotateState.L1;
  // }

  // public Command rotateToL2() {
  //   return run(() -> rotatetol2());
  // }

  // private void rotatetol2() {
  //   rotatePeriodicIO.is_rotate_positional_control = true;
  //   rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_L2_POS;
  //   rotatePeriodicIO.state = RotateState.L2;
  // }

  // public Command rotateToL3() {
  //   return run(() -> rotatetol3());
  // }

  // private void rotatetol3() {
  //   rotatePeriodicIO.is_rotate_positional_control = true;
  //   rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_L3_POS;
  //   rotatePeriodicIO.state = RotateState.L3;
  // }

  // public Command rotateToClimb() {
  //   return run(() -> rotatetoclimb());
  // }

  // private void rotatetoclimb() {
  //   rotatePeriodicIO.is_rotate_positional_control = true;
  //   rotatePeriodicIO.rotate_target = Constants.Rotate.ROTATE_CLIMB_POS;
  //   rotatePeriodicIO.state = RotateState.CLIMB;
  // }
       
  
}
