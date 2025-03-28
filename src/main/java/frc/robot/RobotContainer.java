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
import frc.robot.commands.MoveArmToCollect;
import frc.robot.commands.MoveArmToL1;
import frc.robot.commands.MoveArmToL2;
import frc.robot.commands.MoveArmToL3;
import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj2.command.RunCommand;




public class RobotContainer {
  private final JoystickButton resetHeading;
  private final JoystickButton robotCentric;
  private final JoystickButton slowDriveMode;
  
  private final XboxController CoPilotController;
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
  private final Camera botCam;
  private final MoveArmToCollect armCollect;
  private final MoveArmToL1 armL1;
  private final MoveArmToL2 armL2;
  private final MoveArmToL3 armL3;
  

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    DriverController = new Joystick(0);
    CoPilotController = new XboxController(1);
    cageClimber = new CageClimber();
    algaeArticulate = new AlgaeArticulate();
    algaeClaw = new AlgaeClaw();
    collectorHead = new CollectorHead();
    coralCollector = new CoralCollector();
    armLift = new ArmLift();
    armPivot = new ArmPivot();
    botCam = new Camera();
    armCollect = new MoveArmToCollect(armLift, armPivot);
    armL1 = new MoveArmToL1(armLift, armPivot);
    armL2 = new MoveArmToL2(armLift, armPivot);
    armL3 = new MoveArmToL3(armLift, armPivot);

    DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.log("Robot Initialized");
        
     
    autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Center Start", new AutoCenterStart(swerveSubsystem));
        autoChooser.addOption("Left Start", new AutoLeftStart(swerveSubsystem));
        autoChooser.addOption("Right Start", new AutoRightStart(swerveSubsystem));

        SmartDashboard.putData("Auto Mode", autoChooser);
        

    resetHeading = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kY.value);
    robotCentric = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kX.value);
    slowDriveMode = new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kRightBumper.value);
    

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

    botCam.setDefaultCommand(
      botCam.InitializeBotCam()
    );

    algaeArticulate.setDefaultCommand(
      algaeArticulate.AlgaeUpDown(() -> CoPilotController.getRawAxis(XboxController.Axis.kRightY.value))
    );
    

    algaeClaw.setDefaultCommand(
      algaeClaw.StopClaw()
    );

    armLift.setDefaultCommand(
      armLift.StopLift()
    );

    armPivot.setDefaultCommand(
      armPivot.StopPivot() 
    );
    
    collectorHead.setDefaultCommand(
      collectorHead.ArticulateCoralCollector(() -> CoPilotController.getRawAxis(XboxController.Axis.kLeftX.value) * 0.25)
    );
        
    cageClimber.setDefaultCommand(
      cageClimber.CageClimbStop()
    );

    coralCollector.setDefaultCommand(
      coralCollector.CollectCoralStop()
    );

      
  configureBindings(); 
  }

  private void configureBindings() {
   
  //Drive Controls
    resetHeading.whileTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));

    slowDriveMode.whileTrue(new SwerveController(
        swerveSubsystem,
          () -> -translationLimiter.calculate(DriverController.getRawAxis(translationAxis) * 0.25),
          () -> -strafeLimiter.calculate(DriverController.getRawAxis(strafeAxis) * 0.25),
          () -> rotationLimiter.calculate(DriverController.getRawAxis(rotationAxis) * 0.25),
          () -> robotCentric.getAsBoolean())
    );

  //Algae Controls

    new JoystickButton(CoPilotController, XboxController.Button.kRightBumper.value)
        .whileTrue(algaeClaw.ClawClose());

    new JoystickButton(CoPilotController, XboxController.Button.kLeftBumper.value)
        .whileTrue(algaeClaw.ClawOpen());

  //Coral Arm Controls
    new POVButton(CoPilotController, 0).whileTrue(
          coralCollector.CoralOut()
        );
    new POVButton(CoPilotController, 180).whileTrue(
          coralCollector.CoralIn()
        );

    
        

    //new JoystickButton(CoPilotController, XboxController.Axis.kRightTrigger.value)
    //.whileTrue(coralCollector.CoralIn(() -> CoPilotController.getRawAxis(XboxController.Axis.kRightTrigger.value)));

    //new JoystickButton(CoPilotController, XboxController.Axis.kLeftTrigger.value)
    //.whileTrue(coralCollector.CoralOut(() -> CoPilotController.getRawAxis(XboxController.Axis.kLeftTrigger.value)));

    //new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kA.value)
      //  .onTrue(new MoveArmToCollect(armLift, armPivot));
    
    //new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kB.value)
      //  .onTrue(new MoveArmToL1(armLift, armPivot));
    
    //new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kX.value)
      //  .onTrue(new MoveArmToL2(armLift, armPivot));

    //new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kY.value)
      //  .onTrue(new MoveArmToL3(armLift, armPivot));
    
    //new JoystickButton(CoPilotController, XboxController.Button.kB.value)
        //.onTrue(armL1);

    //new JoystickButton(CoPilotController, XboxController.Button.kX.value)
        //.onTrue(armL2);

    //new JoystickButton(CoPilotController, XboxController.Button.kY.value)
        //.onTrue(armL3);
        
  //Manual Arm Controls
    new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kX.value)
        .whileTrue(armPivot.SimplePivotBack());
        
    new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kY.value)
        .whileTrue(armPivot.SimplePivotForward());
    
    new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kA.value)
        .whileTrue(armLift.SimpleLiftUp());
    
    new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kB.value)
        .whileTrue(armLift.SimpleLiftDown());

  //Climber Controls
    new POVButton(CoPilotController, 90).whileTrue(
          cageClimber.ReadyCageGrabber()
        );

    new POVButton(CoPilotController, 270).whileTrue(
          cageClimber.CageClimb()
        );

  

  //Stop All Subsystems
    new JoystickButton(CoPilotController, XboxController.Button.kStart.value)
    .onTrue(new InstantCommand(() -> {
    cageClimber.CageClimbStop();
    algaeArticulate.StopAlgaeArticulateMotor();
    algaeClaw.StopClaw();
    armLift.StopLift();
    armPivot.StopPivot();
    collectorHead.CollectorHeadStop();
    coralCollector.CollectCoralStop();
    }));
}
    
    //new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kA.value)
    //.onTrue(botCam.InitializeBotCam());

    //new JoystickButton(DriverController, Constants.ControllerRawButtons.XboxController.Button.kB.value)
    //.onTrue(botCam.StopBotCam());

    

   

    

    

    
    
    

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
