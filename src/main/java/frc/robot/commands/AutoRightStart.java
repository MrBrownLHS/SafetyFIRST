// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.ArmIntake;

public class AutoRightStart extends SequentialCommandGroup {
  public AutoRightStart(
    SwerveSubsystem swerve, ArmLift lift, ArmPivot pivot, ArmRotate rotate, ArmIntake intake) {
    addCommands(
      // Run swerveAuto while moving the arm to L3
      //Coral tree is 88 inches from the start line
      new ParallelCommandGroup(
        swerve.swerveAuto(-2.0, 0.0, 0.5, 0.0),  // Move forward 2m
        lift.liftToL3(),                         // Move lift to L3
        pivot.pivotToL3()                        // Move pivot to L3
      ),
      swerve.swerveAuto(0.0, 0, 0, 0.75).withTimeout(1.0),  
      swerve.swerveAuto(0.0, 1.0, 0.5, 0.0),
      swerve.stopSwerveCommand(),
      rotate.CoralRotateAuto(0.25),             // Rotate the coral collector
      intake.CoralOut().withTimeout(5.0)                         // Run the coral out command
    );
  }
}
