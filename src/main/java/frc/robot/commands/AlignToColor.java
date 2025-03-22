// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.DrivetrainConstants;
import frc.robot.subsystems.ColorDetectionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToColor extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final ColorDetectionSubsystem colorDetectionSubsystem;
  private final ProfiledPIDController pid = DrivetrainConstants.kPoseVelocityYController;
  private final LinearFilter filter = LinearFilter.movingAverage(10);

  private final SwerveRequest.RobotCentric drive =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToColor(
      CommandSwerveDrivetrain drivetrain, ColorDetectionSubsystem colorDetectionSubsystem) {
    this.drivetrain = drivetrain;
    this.colorDetectionSubsystem = colorDetectionSubsystem;
    addRequirements(drivetrain, colorDetectionSubsystem);
  }

  @Override
  public void initialize() {
    TrapezoidProfile.Constraints velocityConstraints = new TrapezoidProfile.Constraints(0.5, 0.5);
    pid.setConstraints(velocityConstraints);
    pid.setGoal(0);

    pid.reset(colorDetectionSubsystem.getTX());
    pid.setTolerance(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = colorDetectionSubsystem.getTX();
    double output = pid.calculate(filter.calculate(error));

    drivetrain.setControl(drive.withVelocityY(-output));
    SmartDashboard.putNumber("Color Align PID", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
