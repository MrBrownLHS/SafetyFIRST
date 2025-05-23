// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ArmRotate;


public class MoveArmToCollect extends SequentialCommandGroup {
  
  public MoveArmToCollect(ArmLift lift, ArmPivot pivot, ArmRotate rotate) {
    addCommands(
        lift.liftToCollect(),
        pivot.pivotToCollect(),
        rotate.rotateToCollect()
    );
  }       
}