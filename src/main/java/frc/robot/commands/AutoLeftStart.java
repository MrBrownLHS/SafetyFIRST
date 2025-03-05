// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CollectorArm.CollectorArmState;
import frc.robot.subsystems.CollectorArm;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLeftStart extends SequentialCommandGroup {
  /** Creates a new AutoRoutine. */
  public AutoLeftStart(SwerveSubsystem swerve, CollectorArm collectorArm) {
     addCommands(
      swerve.swerveAuto(-2.0, 0.0, 0.5),  // Move back 2 meters - Tune as needed
      swerve.swerveAuto(0.0, 1.5, 0.5), // Move right 1.5 meters - Tune as needed
      new InstantCommand(() -> collectorArm.moveToState(CollectorArmState.L3), collectorArm),// Move the collector arm to the L3 position
      new RunCommand(() -> collectorArm.AutoCoral(), collectorArm), //Release the coral
      new InstantCommand(() -> collectorArm.stopArm(), collectorArm));  
  }
}
// Build additional auto routines based on field position: https://chatgpt.com/share/67c8b3c0-1dd4-800e-a02a-5414957768bd