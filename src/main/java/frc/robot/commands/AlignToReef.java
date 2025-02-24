// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.DrivetrainConstants;
import frc.robot.constants.Constants.ScoringConstants;
import frc.robot.constants.Constants.ScoringConstants.ScoringPosition;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import java.util.Map;

public class AlignToReef extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ScoringPosition scoringPosition;
  private Pose2d targetPose;
  private final int targetReefId;

  private double MaxSpeed = DrivetrainConstants.kMaxVelocity.in(MetersPerSecond);
  private double MaxAngularRate = DrivetrainConstants.kMaxAngularRate.in(RotationsPerSecond);

  private final PIDController pidX = DrivetrainConstants.kPoseVelocityXController;
  private final PIDController pidY = DrivetrainConstants.kPoseVelocityYController;
  private final PIDController pidTheta = DrivetrainConstants.kPoseThetaController;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05)
      .withRotationalDeadband(MaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   * Creates a new AlignToReef command.
   * 
   * @param drivetrain      The drivetrain subsystem
   * @param scoringPosition The scoring position (LEFT or RIGHT)
   * @param targetReefId    The specific reef ID to target. Use -1 to
   *                        auto-select closest.
   */
  public AlignToReef(CommandSwerveDrivetrain drivetrain, ScoringPosition scoringPosition, int targetReefId) {
    this.drivetrain = drivetrain;
    this.scoringPosition = scoringPosition;
    this.targetReefId = targetReefId;
    addRequirements(drivetrain);
  }

  /**
   * Creates a new AlignToReef command.
   * 
   * @param drivetrain      The drivetrain subsystem
   * @param scoringPosition The scoring position (LEFT or RIGHT)
   */
  public AlignToReef(CommandSwerveDrivetrain drivetrain, ScoringPosition scoringPosition) {
    this(drivetrain, scoringPosition, -1); // Default to -1 if no specific reef ID is given
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose = drivetrain.getState().Pose;
    SmartDashboard.putString("Current Robot Pose", robotPose.toString());

    Map<Integer, Pose2d> reefPoseMap = ScoringConstants.reefPoseMap;

    Pose2d selectedPose = null;
    int selectedTagId = -1;

    if (targetReefId >= 0) {
      for (Map.Entry<Integer, Pose2d> entry : reefPoseMap.entrySet()) {
        int tagId = entry.getKey();
        if (tagId == targetReefId) {
          selectedPose = entry.getValue();
          selectedTagId = tagId;
          break;
        }
      }
      if (selectedPose != null) {
        SmartDashboard.putString("Target Selection Mode", "Specific ID");
      } else {
        SmartDashboard.putString("Target Selection Mode", "ID Not Found, Defaulting to Closest");
      }
    }

    if (selectedPose == null) {
      double minDistance = Double.MAX_VALUE;
      for (Map.Entry<Integer, Pose2d> entry : reefPoseMap.entrySet()) {
        int tagId = entry.getKey();
        Pose2d tagPose = entry.getValue();
        double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
        if (distance < minDistance) {
          minDistance = distance;
          selectedTagId = tagId;
          selectedPose = tagPose;
        }
      }
      SmartDashboard.putString("Target Selection Mode", "Closest Tag");
    }

    SmartDashboard.putNumber("Selected Reef Tag", selectedTagId);

    if (selectedPose == null) {
      System.err.println("No valid reef tag pose available!");
      return;
    }

    if (scoringPosition == ScoringPosition.LEFT) {
      targetPose = selectedPose.transformBy(
          new Transform2d(ScoringConstants.leftReefOffset, new Rotation2d()));
    } else {
      targetPose = selectedPose.transformBy(
          new Transform2d(ScoringConstants.rightReefOffset, new Rotation2d()));
    }
    SmartDashboard.putNumberArray(
        "Target Robot Reef Pose",
        new double[] { targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees() });

    pidX.setTolerance(0.01);
    pidY.setTolerance(0.01);
    pidTheta.setTolerance(Units.degreesToRadians(0.1));

    pidTheta.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    double vx = pidX.calculate(currentPose.getX(), targetPose.getX());
    double vy = pidY.calculate(currentPose.getY(), targetPose.getY());
    double omega = pidTheta.calculate(
        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    vx = MathUtil.clamp(vx, -MaxSpeed, MaxSpeed);
    vy = MathUtil.clamp(vy, -MaxSpeed, MaxSpeed);
    // omega = MathUtil.clamp(omega, -MaxAngularRate, MaxAngularRate);

    SmartDashboard.putNumber("PID vx", vx);
    SmartDashboard.putNumber("PID vy", vy);
    SmartDashboard.putNumber("PID omega", omega);

    final double vxf = vx;
    final double vyf = vy;
    final double omegaf = omega;
    drivetrain.setControl(drive.withVelocityX(-vxf).withVelocityY(-vyf).withRotationalRate(omegaf));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean positionReached = pidX.atSetpoint() && pidY.atSetpoint();
    boolean angleReached = pidTheta.atSetpoint();

    return positionReached && angleReached;
  }
}
