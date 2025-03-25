// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera extends SubsystemBase {
  private UsbCamera botCam;

  public Camera() {
    SmartDashboard.putBoolean("Camera is Active", false);
  }

  private void StartCamera() {
    if (botCam == null) {
      botCam = CameraServer.startAutomaticCapture();
      botCam.setResolution(320, 240);
      botCam.setFPS(15);
      SmartDashboard.putBoolean("Camera is Active", true);
    }
  }

  private void StopCamera() {
    if (botCam != null) {
      botCam.close();
      botCam = null;
      SmartDashboard.putBoolean("Camera is Active", false);
    }
  }

  public Command InitializeBotCam() {
    return new InstantCommand(this::StartCamera, this);
  }

  public Command StopBotCam() {
    return new InstantCommand(this::StopCamera, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Camera is Active", botCam != null);
  }
}
