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

public class AutoCenterStart extends SequentialCommandGroup {
  public AutoCenterStart(
    SwerveSubsystem swerve, ArmLift lift, ArmPivot pivot, ArmRotate rotate, ArmIntake intake){ // CollectorArm collectorArm, CoralCollector coralCollector) {
    addCommands(
      swerve.swerveAuto(-2.0, 0.0, 0.5, 0.0),  // Move forward 2m
      swerve.swerveAuto(0.0, 0, 0, 0.25).withTimeout(2.0),   // Move right 1.5m and rotate slowly
      swerve.stopSwerveCommand(),
      new MoveArmToL1(lift, pivot),
      intake.CoralOut()
      );
  }
}
// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.ArmLift;
// import frc.robot.subsystems.ArmPivot;
// import frc.robot.subsystems.ArmRotate;
// import frc.robot.subsystems.ArmIntake;

// public class AutoCenterStart extends SequentialCommandGroup {
//   public AutoCenterStart(
//     SwerveSubsystem swerve, ArmLift lift, ArmPivot pivot, ArmRotate rotate, ArmIntake intake) {
//     addCommands(
//       // Run swerveAuto while moving the arm to L3
//       new ParallelCommandGroup(
//         swerve.swerveAuto(-2.0, 0.0, 0.5, 0.0),  // Move forward 2m
//         lift.liftToL3(),                         // Move lift to L3
//         pivot.pivotToL3()                        // Move pivot to L3
//       ),
//       swerve.swerveAuto(0.0, 0, 0, 0.25).withTimeout(2.0),   // Move right 1.5m and rotate slowly
//       swerve.stopSwerveCommand(),
//       rotate.CoralRotateAuto(0.15),             // Rotate the coral collector
//       intake.CoralOut()                         // Run the coral out command
//     );
//   }
// }