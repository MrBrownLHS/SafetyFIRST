// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class MoveArmToL2 extends SequentialCommandGroup {
  private final ArmLift lift;
  private final ArmPivot pivot;
  
  public MoveArmToL2(ArmLift lift, ArmPivot pivot) {
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
    return lift.LiftToL2()
               .withTimeout(3) 
               .andThen(() -> lift.StopLift()); 
  }

  private Command PivotCommand(ArmPivot pivot) {
    return pivot.PivotToL2()
                .withTimeout(2) 
                .andThen(() -> pivot.StopPivot());
  }

  private Command StopCommands(ArmLift lift, ArmPivot pivot) {
    return new InstantCommand(() -> {
      pivot.StopPivot();
      lift.StopLift();
    });
  }
}