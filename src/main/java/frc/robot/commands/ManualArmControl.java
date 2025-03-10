// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.CollectorArm;
import frc.robot.utilities.constants.Constants;


public class ManualArmControl extends Command {
  private final CollectorArm collectorArm;
  private final XboxController coPilotController;
  
  private static final double LIFT_INCREMENT = 1.0;  // Inches per press
  private static final double PIVOT_INCREMENT = 2.0; // Degrees per press

  private double manualLiftPosition;
  private double manualPivotAngle;
  
  public ManualArmControl(CollectorArm collectorArm, XboxController coPilotController) {
    this.collectorArm = collectorArm;
    this.coPilotController = coPilotController;
        addRequirements(collectorArm);
    
  }


  @Override
  public void initialize() {
    manualLiftPosition = collectorArm.getLiftHeightInches();
    manualPivotAngle = collectorArm.getPivotAngleDegrees();
  }

  
  @Override
  public void execute() {// If D-Pad North is pressed, override preset setpoints and allow manual control
    if (coPilotController.getPOV() == 0) { 
        
        // Lift Control with Bumpers
        if (coPilotController.getLeftBumper()) {
            manualLiftPosition -= LIFT_INCREMENT; // Decrease lift height
        } 
        if (coPilotController.getRightBumper()) {
            manualLiftPosition += LIFT_INCREMENT; // Increase lift height
        }

        // Pivot Control with Triggers
        double leftTrigger = coPilotController.getLeftTriggerAxis();
        double rightTrigger = coPilotController.getRightTriggerAxis();
        
        if (leftTrigger > 0.1) {
            manualPivotAngle -= PIVOT_INCREMENT * leftTrigger; // Rotate arm down
        } 
        if (rightTrigger > 0.1) {
            manualPivotAngle += PIVOT_INCREMENT * rightTrigger; // Rotate arm up
        }

        // Ensure the arm stays within safe limits
        manualLiftPosition = MathUtil.clamp(manualLiftPosition,
            Constants.CollectorArmConstants.LIFT_MIN_HEIGHT,
            Constants.CollectorArmConstants.LIFT_MAX_HEIGHT);

        manualPivotAngle = MathUtil.clamp(manualPivotAngle,
            Constants.CollectorArmConstants.PIVOT_MIN_ANGLE,
            Constants.CollectorArmConstants.PIVOT_MAX_ANGLE);

        // Move the arm to the manually controlled positions
        collectorArm.setLiftPosition(manualLiftPosition);
        collectorArm.setPivotPosition(manualPivotAngle);
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
