// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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

import frc.robot.subsystems.CollectorHead;
import frc.robot.subsystems.CoralCollector;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.CageClimber;
import frc.robot.subsystems.AlgaeArticulate;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.ArmPivot;
import frc.robot.commands.AutoCenterStart;
import frc.robot.commands.AutoLeftStart;
import frc.robot.commands.AutoRightStart;




public class RobotContainer {
  private final JoystickButton resetHeading;
  private final JoystickButton robotCentric;

  private final Joystick CoPilotController;
  private final CageClimber cageClimber;
  private final AlgaeArticulate algaeArticulate;
  private final AlgaeClaw algaeClaw;
  private final CollectorHead collectorHead;
  private final CoralCollector coralCollector;
  private final ArmLift armLift;
  private final ArmPivot armPivot;
  
  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationAxis;
  
  
  private final Joystick DriverController;
  private final SwerveSubsystem swerveSubsystem;

  private final SendableChooser<Command> autoChooser;
  private boolean tuningMode = false;

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    DriverController = new Joystick(0);
    CoPilotController = new Joystick(1);
    cageClimber = new CageClimber();
    algaeArticulate = new AlgaeArticulate();
    algaeClaw = new AlgaeClaw();
    collectorHead = new CollectorHead();
    coralCollector = new CoralCollector();
    armLift = new ArmLift();
    armPivot = new ArmPivot();
    DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.log("Robot Initialized");
        SmartDashboard.putBoolean("Tuning Mode", false);
     
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

   
    algaeArticulate.setDefaultCommand(
      algaeArticulate.AlgaeUpDown(
      () -> CoPilotController.getRawAxis(XboxController.Axis.kRightY.value) * 0.25
      )
    );

    algaeClaw.setDefaultCommand(
      algaeClaw.stopClaw()
    );

    armLift.setDefaultCommand(
      armLift.StopLift()
    );

    armPivot.setDefaultCommand(
      armPivot.StopPivot()
    );
    
   collectorHead.setDefaultCommand(
     collectorHead.ArticulateCoralCollector(
      () -> CoPilotController.getRawAxis(XboxController.Axis.kLeftX.value) * 0.1
      )
    );

        
    cageClimber.setDefaultCommand(
      cageClimber.CageClimbStop()
    );

      
  configureBindings(); 
  }

  private void configureBindings() {
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    new JoystickButton(DriverController, XboxController.Button.kRightBumper.value)
    .whileTrue(new RunCommand(() -> {
        swerveSubsystem.drive(
            new edu.wpi.first.math.geometry.Translation2d(
                -translationLimiter.calculate(DriverController.getRawAxis(translationAxis) * 0.25),
                -strafeLimiter.calculate(DriverController.getRawAxis(strafeAxis) * 0.25)
            ),
            rotationLimiter.calculate(DriverController.getRawAxis(rotationAxis) * 0.25),
            robotCentric.getAsBoolean(),
            false // Adjust this boolean as needed based on your method's requirements
        );
    }, swerveSubsystem));

    new JoystickButton(CoPilotController, XboxController.Axis.kLeftTrigger.value)
    .whileTrue(new RunCommand(() -> coralCollector.CollectCoral(() -> -0.5), coralCollector));

    new JoystickButton(CoPilotController, XboxController.Axis.kRightTrigger.value)
    .whileTrue(new RunCommand(() -> coralCollector.CollectCoral(() -> 0.5), coralCollector));


    new JoystickButton(CoPilotController, XboxController.Button.kRightBumper.value)
    .whileTrue(algaeClaw.SimpleClawClose());

    new JoystickButton(CoPilotController, XboxController.Button.kLeftBumper.value)
    .whileTrue(algaeClaw.SimpleClawOpen());

    new JoystickButton(CoPilotController, XboxController.Button.kX.value)
    .whileTrue(armPivot.ManualPivotToMin());
    
    new JoystickButton(CoPilotController, XboxController.Button.kY.value)
    .whileTrue(armPivot.ManualPivotToMax());

    new JoystickButton(CoPilotController, XboxController.Button.kA.value)
    .whileTrue(armLift.ManualLiftToMax());

    new JoystickButton(CoPilotController, XboxController.Button.kB.value)
    .whileTrue(armLift.ManualLiftToMin());

    new POVButton(CoPilotController, 90).whileTrue(
      cageClimber.ReadyCageGrabber()
    );

    new POVButton(CoPilotController, 270).whileTrue(
      cageClimber.CageClimb()
    );

    new POVButton(CoPilotController, 180)
      .onTrue(new InstantCommand(() -> {
      cageClimber.CageClimbStop();
      algaeArticulate.stopAlgaeArticulateMotor();
      algaeClaw.stopClaw();
      armLift.StopLift();
      armPivot.StopPivot();
      collectorHead.CollectorHeadStop();
      coralCollector.CollectCoralStop();
      }));
    }
    

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
