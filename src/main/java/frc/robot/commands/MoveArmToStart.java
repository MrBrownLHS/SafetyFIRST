// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToStart extends SequentialCommandGroup {
  private final ArmLift lift;
  private final ArmPivot pivot;

  public MoveArmToStart(ArmLift lift, ArmPivot pivot) {
    this.lift = lift;
    this.pivot = pivot;
    addRequirements(lift, pivot);
    addCommands(
        LiftCommand(lift),
        PivotCommand(pivot),
        StopCommands(lift, pivot)
    );
  }

  private Command LiftCommand(ArmLift lift) {
    return lift.liftToStart()
              .andThen(() -> {
                lift.stopLift();
              });
  }

  private Command PivotCommand(ArmPivot pivot) {
    return pivot.pivotToStart()
              .andThen(() -> {
                pivot.stopPivot();
              });
  }

  private Command StopCommands(ArmLift lift, ArmPivot pivot) {
    return new InstantCommand(() -> {
        pivot.stopPivot();
        lift.stopLift();
      });
  }
}
