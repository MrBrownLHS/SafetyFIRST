// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ArmRotate;
import frc.robot.subsystems.ArmIntake;
import frc.robot.commands.MoveArmToL3Command;

public class AutoCenterStart extends SequentialCommandGroup {
  public AutoCenterStart(
    SwerveSubsystem swerve, ArmLift lift, ArmPivot pivot, ArmRotate rotate, ArmIntake intake){ // CollectorArm collectorArm, CoralCollector coralCollector) {
    addCommands(
      swerve.swerveAuto(-2.0, 0.0, 0.5, 0.0),  // Move forward 2m
      swerve.swerveAuto(0.0, 0, 0, 0.25),   // Move right 1.5m and rotate slowly
      swerve.stopSwerveCommand(),
      new MoveArmToL3Command(lift, pivot, rotate, intake);
      );
  }
}
