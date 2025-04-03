// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.commands.RunCoralCollector;
import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj2.command.RunCommand;




public class RobotContainer {
  private final JoystickButton resetHeading;
  private final JoystickButton robotCentric;
  private final JoystickButton slowDriveMode;
  
  private final XboxController CoPilotController = new XboxController(1);
  private static final CommandXboxController CopilotCommandController = new CommandXboxController(1);
  private final CageClimber cageClimber = new CageClimber();
  private final AlgaeArticulate algaeArticulate = new AlgaeArticulate();
  private final AlgaeClaw algaeClaw = new AlgaeClaw();
  private final CollectorHead collectorHead = new CollectorHead();
  private final CoralCollector coralCollector = new CoralCollector();
  private final ArmLift armLift = new ArmLift();
  private final ArmPivot armPivot = new ArmPivot();
  
  private final int translationAxis;
  private final int strafeAxis;
  private final int rotationAxis;
  
  private final Joystick DriverController= new Joystick(0);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final SendableChooser<Command> autoChooser;
  private final Camera botCam = new Camera();
  private final MoveArmToCollect armCollect = new MoveArmToCollect(armLift, armPivot);
  private final MoveArmToL1 armL1 = new MoveArmToL1(armLift, armPivot);
  private final MoveArmToL2 armL2 = new MoveArmToL2(armLift, armPivot);
  private final MoveArmToL3 armL3 = new MoveArmToL3(armLift, armPivot);
  

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);


  public RobotContainer() {
   
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

    // armLift.setDefaultCommand(
    //   new InstantCommand(() -> {}));

    // armPivot.setDefaultCommand(
    //   new InstantCommand(() -> {}));
    
    collectorHead.setDefaultCommand(
      collectorHead.ArticulateCoralCollector(() -> CoPilotController.getRawAxis(XboxController.Axis.kLeftX.value) * 0.25)
    );
        
    cageClimber.setDefaultCommand(
      cageClimber.CageClimbStop()
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

 
    CopilotCommandController.axisMagnitudeGreaterThan(2, 0.5).whileTrue(new RunCoralCollector(coralCollector, 0.25));
    CopilotCommandController.axisMagnitudeGreaterThan(3, 0.5).whileTrue(new RunCoralCollector(coralCollector, -0.25));


    new JoystickButton(CoPilotController, XboxController.Button.kA.value)
       .onTrue(armCollect);
    
    new JoystickButton(CoPilotController, XboxController.Button.kB.value)
       .onTrue(armL1);
    
    new JoystickButton(CoPilotController, XboxController.Button.kX.value)
       .onTrue(armL2);

    new JoystickButton(CoPilotController, XboxController.Button.kY.value)
       .onTrue(armL3);
    
         
  //Manual Arm Controls
    // new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kX.value)
    //     .whileTrue(armPivot.SimplePivotBack());
        
    // new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kY.value)
    //     .whileTrue(armPivot.SimplePivotForward());
    
    // new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kA.value)
    //     .whileTrue(armLift.SimpleLiftUp());
    
    // new JoystickButton(CoPilotController, Constants.ControllerRawButtons.XboxController.Button.kB.value)
    //     .whileTrue(armLift.SimpleLiftDown());

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

    new JoystickButton(CoPilotController, XboxController.Button.kA.value)
    .onTrue(new InstantCommand(() -> System.out.println("A button pressed")));
}
    


    

   

    

    

    
    
    

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
