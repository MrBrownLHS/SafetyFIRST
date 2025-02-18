// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.SwerveController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.constants.Constants;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.CollectorArm.CollectorArmState;

public class RobotContainer {
  private final JoystickButton resetHeading;
  private final JoystickButton robotCentric;

  private final XboxController coPilotController;
  private final CollectorArm collectorArm;

  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationAxis;

  private final Joystick DriverController;
  private final SwerveSubsystem swerveSubsystem;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    DriverController = new Joystick(0);
    coPilotController = new XboxController(1);
    collectorArm = new CollectorArm();

    resetHeading = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kY.value);
    robotCentric = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kX.value);

    translationAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftY.value;
    strafeAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftX.value;
    rotationAxis = Constants.ControllerRawButtons.XboxController.Axis.kRightX.value;

    swerveSubsystem.setDefaultCommand(new SwerveController(
            swerveSubsystem,
            () -> DriverController.getRawAxis(translationAxis) * 0.25,
            () -> DriverController.getRawAxis(strafeAxis) * 0.25,
            () -> -DriverController.getRawAxis(rotationAxis) * 0.25,
            () -> robotCentric.getAsBoolean()) // lambda probably not needed but why not
    );

    configureBindings();
  }

  private void configureBindings() {
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    new JoystickButton(coPilotController, XboxController.Button.kA.value)
    .onTrue(new InstantCommand(() -> collectorArm.moveToState(CollectorArmState.START), collectorArm));

    new JoystickButton(coPilotController, XboxController.Button.kB.value)
    .onTrue(new InstantCommand(() -> collectorArm.moveToState(CollectorArmState.FLOOR), collectorArm));

    new JoystickButton(coPilotController, XboxController.Button.kX.value)
    .onTrue(new InstantCommand(() -> collectorArm.moveToState(CollectorArmState.COLLECT), collectorArm));

    new JoystickButton(coPilotController, XboxController.Button.kY.value)
    .onTrue(new InstantCommand(() -> collectorArm.moveToState(CollectorArmState.L3), collectorArm));

    new JoystickButton(coPilotController, XboxController.Button.kLeftBumper.value)
    .onTrue(new InstantCommand(() -> collectorArm.moveToState(CollectorArmState.L2), collectorArm));

    new JoystickButton(coPilotController, XboxController.Button.kRightBumper.value)
    .onTrue(new InstantCommand(() -> collectorArm.moveToState(CollectorArmState.MAX), collectorArm));

    new JoystickButton(coPilotController, XboxController.Button.kStart.value)
    .onTrue(new InstantCommand(collectorArm::stopArm, collectorArm));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
