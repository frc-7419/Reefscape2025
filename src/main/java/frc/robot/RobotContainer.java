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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaeClampWithJoystick;
import frc.robot.commands.AntiTip;
import frc.robot.commands.ToPose;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CameraConfig;
import frc.robot.constants.Constants.DrivetrainConstants;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.PhotonvisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.AlgaeClampSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
  private double MaxSpeed = DrivetrainConstants.kMaxVelocity.in(MetersPerSecond);
  private double MaxAngularRate = DrivetrainConstants.kMaxAngularRate.in(RotationsPerSecond);

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.05)
          .withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ToPose toPose = new ToPose(drivetrain);

  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final AntiTip antiTip = new AntiTip(drivetrain, elevator);
  private final WristSubsystem wrist = new WristSubsystem();
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

  private final AlgaeClampSubsystem algaeClampSubsystem = new AlgaeClampSubsystem();
  private final AlgaeClampWithJoystick algaeClampWithJoystick =
      new AlgaeClampWithJoystick(algaeClampSubsystem, operator);

  public RobotContainer() {
    photonvision = new PhotonvisionSubsystem(cameraConfigs);

    configureBindings();
    SmartDashboard.putBoolean("isConfigured", AutoBuilder.isConfigured());

    algaeClampSubsystem.setDefaultCommand(algaeClampWithJoystick);
    // antiTip.schedule();
    
    private final Command elevatorToL1 =
        elevator.setPosition(Meters.of(Constants.ScoringConstants.elevatorSetPointL1));
    private final Command elevatorToL2 =
        elevator.setPosition(Meters.of(Constants.ScoringConstants.elevatorSetPointL2));
    private final Command elevatorToL3 =
        elevator.setPosition(Meters.of(Constants.ScoringConstants.elevatorSetPointL3));
    private final Command elevatorToL4 =
        elevator.setPosition(Meters.of(Constants.ScoringConstants.elevatorSetPointL4));
    private final Command wristL1 =
        wrist.setAngle(Degrees.of(Constants.ScoringConstants.wristSetPointL1));
    private final Command wristL2 =
        wrist.setAngle(Degrees.of(Constants.ScoringConstants.wristSetPointL2));
    private final Command wristL3 =
        wrist.setAngle(Degrees.of(Constants.ScoringConstants.wristSetPointL3));
    private final Command wristL4 =
        wrist.setAngle(Degrees.of(Constants.ScoringConstants.wristSetPointL4));
    private final Command scoreL1 = new ParallelCommandGroup(elevatorToL1, wristL1);
    private final Command scoreL2 = new ParallelCommandGroup(elevatorToL2, wristL2);
    private final Command scoreL3 = new ParallelCommandGroup(elevatorToL3, wristL3);
    private final Command scoreL4 = new ParallelCommandGroup(elevatorToL4, wristL4);
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

    operator.a().whileTrue(elevator.setPosition(Meters.of(0)));

    operator.b().whileTrue(scoreL4);

    operator.x().whileTrue(scoreL2);

    operator.y().whileTrue(scoreL3);

    operator.leftBumper().whileTrue(scoreL1);

    elevator.setDefaultCommand(elevator.joystickControl(operator.getLeftY()));

    wrist.setDefaultCommand(wrist.joystickControl(operator.getRightY()));
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return new PathPlannerAuto("Example Path");
  }
}
