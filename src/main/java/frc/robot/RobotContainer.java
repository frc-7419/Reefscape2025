// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AntiTip;
import frc.robot.commands.ToPose;
import frc.robot.constants.Constants.CameraConfig;
import frc.robot.constants.Constants.DrivetrainConstants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.PhotonvisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RobotContainer {
  private double MaxSpeed = DrivetrainConstants.kMaxVelocity.in(MetersPerSecond);
  private double MaxAngularRate = DrivetrainConstants.kMaxAngularRate.in(RotationsPerSecond);

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ToPose toPose = new ToPose(drivetrain);
  // private final SendableChooser<Command> autoChooser;
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final AntiTip antiTip = new AntiTip(drivetrain, elevator);
  public final PhotonvisionSubsystem photonvision;
  private final CameraConfig photonCamOne =
      new CameraConfig("Photon_Vision_Cam_1", VisionConstants.kRobotToCamOne);
  private final CameraConfig photonCamTwo =
      new CameraConfig("Photon_Vision_Cam_2", VisionConstants.kRobotToCamOne);
  private final List<CameraConfig> cameraConfigs =
      new ArrayList<CameraConfig>() {
        {
          add(photonCamOne);
          add(photonCamTwo);
        }
      };

  public RobotContainer() {
    photonvision = new PhotonvisionSubsystem(cameraConfigs);

    configureBindings();
    SmartDashboard.putBoolean("isConfigured", AutoBuilder.isConfigured());
    // antiTip.schedule();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -driver.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // driver
    // .pov(0)
    // .whileTrue(
    // drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // driver
    // .pov(180)
    // .whileTrue(
    // drivetrain.applyRequest(() ->
    // forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    driver.x().whileTrue(toPose);

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

    operator.a().whileTrue(elevator.setPosition(Inches.of(0)));

    elevator.setDefaultCommand(elevator.joystickControl(operator.getRightY()));
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return new PathPlannerAuto("Example Path");
  }
}
