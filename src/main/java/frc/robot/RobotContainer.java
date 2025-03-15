// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.commands.SwerveController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utilities.constants.Constants;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.CollectorHead;
import frc.robot.subsystems.CoralCollector;
import frc.robot.subsystems.CollectorArm.CollectorArmState;
import frc.robot.subsystems.CageClimber;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.commands.AutoCenterStart;
import frc.robot.commands.AutoLeftStart;
import frc.robot.commands.AutoRightStart;
import frc.robot.commands.ManualArmControl;


public class RobotContainer {
  private final JoystickButton resetHeading;
  private final JoystickButton robotCentric;

  private final Joystick CoPilotController;
  private final CollectorArm collectorArm;
  private final CageClimber cageClimber;
  private final AlgaeCollector algaeCollector;
  private final CollectorHead collectorHead;
  private final CoralCollector coralCollector;
  
  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationAxis;
  
  
  private final Joystick DriverController;
  private final SwerveSubsystem swerveSubsystem;

  private final SendableChooser<Command> autoChooser;

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    DriverController = new Joystick(0);
    CoPilotController = new Joystick(1);
    cageClimber = new CageClimber();
    algaeCollector = new AlgaeCollector();
    collectorHead = new CollectorHead();
    coralCollector = new CoralCollector();
    collectorArm = new CollectorArm();

     autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Center Start", new AutoCenterStart(swerveSubsystem));
        autoChooser.addOption("Left Start", new AutoLeftStart(swerveSubsystem));
        autoChooser.addOption("Right Start", new AutoRightStart(swerveSubsystem));

        SmartDashboard.putData("Auto Mode", autoChooser);
        
        

    resetHeading = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kY.value);
    robotCentric = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kX.value);

    translationAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftY.value;
    strafeAxis = Constants.ControllerRawButtons.XboxController.Axis.kLeftX.value;
    rotationAxis = Constants.ControllerRawButtons.XboxController.Axis.kRightX.value;
    
    swerveSubsystem.setDefaultCommand(new SwerveController(
            swerveSubsystem,
            () -> -translationLimiter.calculate(DriverController.getRawAxis(translationAxis) * 0.75),
            () -> -strafeLimiter.calculate(DriverController.getRawAxis(strafeAxis) * 0.75),
            () -> rotationLimiter.calculate(DriverController.getRawAxis(rotationAxis) * 0.75),
            () -> robotCentric.getAsBoolean()) // lambda probably not needed but why not
    );

    collectorHead.setDefaultCommand(
      collectorHead.ArticulateCoralCollector(
        () -> CoPilotController.getRawAxis(XboxController.Axis.kLeftY.value)
      )
    );

    coralCollector.setDefaultCommand(
      coralCollector.CollectCoral(
        () -> CoPilotController.getRawAxis(XboxController.Axis.kLeftX.value)
      )
    );

    algaeCollector.setDefaultCommand(
      algaeCollector.ArticulateAndCollectAlgae(
        () -> CoPilotController.getRawAxis(XboxController.Axis.kRightY.value),
        () -> CoPilotController.getRawAxis(XboxController.Axis.kRightX.value)
      )
    );

    cageClimber.setDefaultCommand(
      cageClimber.CageClimbStop());
    
  configureBindings(); 
  }

  private void configureBindings() {
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    //new JoystickButton(CoPilotController, XboxController.Button.kA.value)
    //.onTrue(coralCollectorArm.moveToStateCommand(CollectorArmState.COLLECT, 3));

    //new JoystickButton(CoPilotController, XboxController.Button.kB.value)
    //.onTrue(coralCollectorArm.moveToStateCommand(CollectorArmState.L1, 3));

    //new JoystickButton(CoPilotController, XboxController.Button.kX.value)
    //.onTrue(coralCollectorArm.moveToStateCommand(CollectorArmState.L2, 3));

    //new JoystickButton(CoPilotController, XboxController.Button.kY.value)
    //.onTrue(coralCollectorArm.moveToStateCommand(CollectorArmState.L3, 3));

    new JoystickButton(CoPilotController, XboxController.Button.kStart.value)
    .onTrue(new InstantCommand(collectorArm::stopArm, collectorArm));

    new JoystickButton(CoPilotController, XboxController.Button.kBack.value)
    .onTrue(new InstantCommand(algaeCollector::StopAlgae));

    new POVButton(CoPilotController, 90).whileTrue(
      cageClimber.ReadyCageGrabber()
    );

    new POVButton(CoPilotController, 270).whileTrue(
      cageClimber.CageClimb()
    );

    new POVButton(CoPilotController, 180).onTrue(
      cageClimber.CageClimbStop()
    );

    new POVButton(CoPilotController, 0).whileTrue(
      new ManualArmControl(collectorArm, CoPilotController)
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
