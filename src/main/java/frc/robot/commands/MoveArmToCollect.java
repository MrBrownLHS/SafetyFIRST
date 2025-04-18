// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;




public class MoveArmToCollect extends SequentialCommandGroup {
  
  public MoveArmToCollect(ArmLift lift, ArmPivot pivot) {
    addCommands(
        LiftCommand(lift),
        PivotCommand(pivot)
    );
        
  }

  private Command LiftCommand(ArmLift lift) {
    return lift.liftToCollect()
              .andThen(() -> {
                lift.stopLift();
              });
  }

  private Command PivotCommand(ArmPivot pivot) {
    return pivot.pivotToCollect()
              .andThen(() -> {
                pivot.stopPivot();
              });
  }

  
}