// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ArmRotate;

public class MoveArmToL2 extends SequentialCommandGroup {
  
  public MoveArmToL2(ArmLift lift, ArmPivot pivot, ArmRotate rotate) {
    addCommands(
        lift.liftToL2(),
        pivot.pivotToL2(), 
        rotate.rotateToL2()
    );
  }       
}
