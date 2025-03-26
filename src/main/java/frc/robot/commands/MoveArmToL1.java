// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class MoveArmToL1 extends SequentialCommandGroup {
  public MoveArmToL1(ArmLift lift, ArmPivot pivot) {
    addCommands(
      LiftCommand(lift),
      PivotCommand(pivot),
      StopCommands(lift, pivot)
    );
  }

  private Command LiftCommand(ArmLift lift) {
    return lift.LiftToL1()
               .withTimeout(3) 
               .andThen(() -> lift.StopLift()); 
  }

  private Command PivotCommand(ArmPivot pivot) {
    return pivot.PivotToL1()
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

//*A different command structure to try */
//public MoveArmToL1(ArmLift lift, ArmPivot pivot) {
  //addCommands(
    //lift.LiftToL1().until(lift::isAtTarget),  // 🚀 Wait until Lift is at L1
    //pivot.PivotToL1().until(pivot::isAtTarget), // 🚀 Wait until Pivot is at L1
    //new InstantCommand(() -> { // Ensure Stop Commands
      //lift.StopLift();
      //pivot.StopPivot();
    //})
  //);
//}
//}