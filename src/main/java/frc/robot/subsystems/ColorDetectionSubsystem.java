// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class ColorDetectionSubsystem extends SubsystemBase {
  private final String limelight;
  private static final double ALIGN_THRESHOLD = 1;
  private static final double MIN_TARGET_AREA = 2.0;

  public ColorDetectionSubsystem(String limelightName) {
    this.limelight = limelightName;
    LimelightHelpers.setLEDMode_ForceOff(limelight);
  }

  public double getTX() {
    return LimelightHelpers.getTX(limelight);
  }

  public double getTargetArea() {
    return LimelightHelpers.getTA(limelight);
  }

  public boolean hasValidTarget() {
    return getTargetArea() > MIN_TARGET_AREA;
  }

  public boolean isAligned() {
    if (!hasValidTarget()) {
      return false;
    }
    return Math.abs(getTX()) < ALIGN_THRESHOLD;
  }

  public void enableLED() {
    LimelightHelpers.setLEDMode_ForceOn(limelight);
  }

  public void disableLED() {
    LimelightHelpers.setLEDMode_ForceOff(limelight);
  }

  public void blinkLED() {
    LimelightHelpers.setLEDMode_ForceBlink(limelight);
  }

  public void setLEDModePipelineControl() {
    LimelightHelpers.setLEDMode_PipelineControl(limelight);
  }

  public void setPipeline(int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(limelight, pipelineIndex);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Reef TX", getTX());
    SmartDashboard.putBoolean("Is Aligned (Color)", isAligned());
  }
}
