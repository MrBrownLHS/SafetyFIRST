// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
//import frc.robot.subsystems.ArmRotate;


public class MoveArmToClimb extends SequentialCommandGroup {
  
  public MoveArmToClimb(ArmLift lift, ArmPivot pivot) {
    addCommands(
      lift.liftToClimb(),
      pivot.pivotToClimb()
      //rotate.rotateToClimb()
    );
  }       
}
