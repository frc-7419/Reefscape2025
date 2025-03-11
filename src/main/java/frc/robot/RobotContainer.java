// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignAndScore;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.AutoIntakeCoral;
import frc.robot.commands.ScoringSetpoints;
import frc.robot.constants.Constants.CameraConfig;
import frc.robot.constants.Constants.DrivetrainConstants;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.ScoringConstants.ScoringPosition;
import frc.robot.constants.Constants.ScoringConstants.ScoringSetpoint;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
import frc.robot.subsystems.elevator.RunElevatorWithJoystick;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;
import frc.robot.subsystems.intake.WristIntakeSubsystem;
import frc.robot.subsystems.wrist.RunWristWithJoystick;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToPosition;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class RobotContainer {
  private double MaxSpeed = DrivetrainConstants.kMaxVelocity.in(MetersPerSecond);
  private double MaxAngularRate = DrivetrainConstants.kMaxAngularRate.in(RotationsPerSecond);

  private WristIntakeSubsystem wristIntakeSubsystem = new WristIntakeSubsystem();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private RunIntakeWithJoystick runIntakeWithJoystick =
      new RunIntakeWithJoystick(wristIntakeSubsystem, operator);
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * DrivetrainConstants.kTranslationDeadband)
          .withRotationalDeadband(
              MaxAngularRate * DrivetrainConstants.kRotationDeadband) // Add a 5%
          // deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private WristSubsystem wrist = new WristSubsystem();
  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(wrist::getPosition, drivetrain::getPigeon2);

  public final VisionSubsystem photonvision;
  private final CameraConfig photonCamOne =
      new CameraConfig("Cam1", VisionConstants.kRobotToCamOne);
  private final CameraConfig photonCamTwo =
      new CameraConfig("Cam2", VisionConstants.kRobotToCamTwo);

  private final SendableChooser<Command> autoChooser;

  private final List<CameraConfig> cameraConfigs =
      new ArrayList<CameraConfig>() {
        {
          add(photonCamOne);
          add(photonCamTwo);
        }
      };

  private boolean slowMode = false;

  private boolean coral = true;

  public RobotContainer() {
    photonvision = new VisionSubsystem(cameraConfigs);

    configureBindings();
    SmartDashboard.putBoolean("isConfigured", AutoBuilder.isConfigured());

    SmartDashboard.putBoolean("Coral Mode", coral);

    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auton Path", autoChooser);
  }

  private final Command alignAndScoreL1Left =
      new AlignAndScore(
          drivetrain,
          elevator,
          wrist,
          wristIntakeSubsystem,
          ScoringPosition.LEFT,
          ScoringSetpoint.L1);
  private final Command alignAndScoreL1Right =
      new AlignAndScore(
          drivetrain,
          elevator,
          wrist,
          wristIntakeSubsystem,
          ScoringPosition.RIGHT,
          ScoringSetpoint.L1);

  private final Command alignAndScoreL2Left =
      new AlignAndScore(
          drivetrain,
          elevator,
          wrist,
          wristIntakeSubsystem,
          ScoringPosition.LEFT,
          ScoringSetpoint.L2);
  private final Command alignAndScoreL2Right =
      new AlignAndScore(
          drivetrain,
          elevator,
          wrist,
          wristIntakeSubsystem,
          ScoringPosition.RIGHT,
          ScoringSetpoint.L2);

  private final Command alignAndScoreL3Left =
      new AlignAndScore(
          drivetrain,
          elevator,
          wrist,
          wristIntakeSubsystem,
          ScoringPosition.LEFT,
          ScoringSetpoint.L3);
  private final Command alignAndScoreL3Right =
      new AlignAndScore(
          drivetrain,
          elevator,
          wrist,
          wristIntakeSubsystem,
          ScoringPosition.RIGHT,
          ScoringSetpoint.L3);

  private final Command alignAndScoreL4Left =
      new AlignAndScore(
          drivetrain,
          elevator,
          wrist,
          wristIntakeSubsystem,
          ScoringPosition.LEFT,
          ScoringSetpoint.L4);
  private final Command alignAndScoreL4Right =
      new AlignAndScore(
          drivetrain,
          elevator,
          wrist,
          wristIntakeSubsystem,
          ScoringPosition.RIGHT,
          ScoringSetpoint.L4);

  private void registerNamedCommands() {
    Map<String, Command> namedCommands = new HashMap<>();
    namedCommands.put("AlignAndScoreL1Left", alignAndScoreL1Left);
    namedCommands.put("AlignAndScoreL1Right", alignAndScoreL1Right);

    namedCommands.put("AlignAndScoreL2Left", alignAndScoreL2Left);
    namedCommands.put("AlignAndScoreL2Right", alignAndScoreL2Right);

    namedCommands.put("AlignAndScoreL3Left", alignAndScoreL3Left);
    namedCommands.put("AlignAndScoreL3Right", alignAndScoreL3Right);

    namedCommands.put("AlignAndScoreL4Left", alignAndScoreL4Left);
    namedCommands.put("AlignAndScoreL4Right", alignAndScoreL4Right);

    namedCommands.put("IntakeCoral", new AutoIntakeCoral(wristIntakeSubsystem, elevator, wrist));

    NamedCommands.registerCommands(namedCommands);
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -driver.getLeftY()
                            * MaxSpeed
                            * (driver.rightBumper().getAsBoolean() ? 0.2 : 1)) // Drive
                    // forward
                    // with
                    // negative Y (forward)
                    .withVelocityY(
                        -driver.getLeftX()
                            * MaxSpeed
                            * (driver.rightBumper().getAsBoolean() ? 0.2 : 1)) // Drive
                    // left
                    // with
                    // negative
                    // X
                    // (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive
            // counterclockwise
            // with
            // negative
            // X
            // (left)
            ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    driver.leftTrigger(0.2).whileTrue(new AlignToReef(drivetrain, ScoringPosition.LEFT, true));
    driver.rightTrigger(0.2).whileTrue(new AlignToReef(drivetrain, ScoringPosition.RIGHT, true));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.

    driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
    /*
     * operator.a().whileTrue(elevator.setPosition(Meters.of(0)));
     *
     * operator.b().whileTrue(scoreL4);
     *
     * operator.x().whileTrue(scoreL2);
     *
     * operator.y().whileTrue(scoreL3);
     *
     * operator.leftBumper().whileTrue(scoreL1);
     */
    // operator.x().whileTrue(new ElevatorPIDTest(elevator));
    operator.back().onTrue(new RunCommand(() -> elevator.zeroEncoder(), elevator));
    operator.leftBumper().whileTrue(new AutoIntakeCoral(wristIntakeSubsystem, elevator, wrist));
    // operator.y().whileTrue(new WristPIDTest(wristSubsystem));
    operator.b().whileTrue(new WristToPosition(wrist, Rotations.of(0.46)));
    operator.a().whileTrue(new WristToPosition(wrist, Rotations.of(0.38)));
    operator.x().onTrue(alignAndScoreL4Left);

    wristIntakeSubsystem.setDefaultCommand(runIntakeWithJoystick);
    wrist.setDefaultCommand(new RunWristWithJoystick(wrist, () -> operator.getRightY() * 0.15));

    elevator.setDefaultCommand(new MaintainElevatorPosition(elevator));
    DoubleSupplier elevatorPowerSupplier = () -> operator.getLeftY();
    new Trigger(
            () ->
                (Math.abs(elevatorPowerSupplier.getAsDouble())
                    > ElevatorConstants.joystickDeadband))
        .whileTrue(
            new RunElevatorWithJoystick(elevator, elevatorPowerSupplier, wrist::getPosition));

    wristIntakeSubsystem.setDefaultCommand(
        new RunIntakeWithJoystick(wristIntakeSubsystem, operator));

    operator
        .start()
        .onTrue(
            new InstantCommand(
                () -> {
                  coral = !coral;
                  SmartDashboard.putBoolean("Coral Mode", coral);
                }));
    Set<Subsystem> scoringDependencies = new HashSet<>(Arrays.asList(elevator, wrist));

    operator
        .povUp()
        .whileTrue(
            new ConditionalCommand(
                Commands.defer(
                    () -> new ScoringSetpoints(elevator, wrist, ScoringSetpoint.L4, true),
                    scoringDependencies),
                Commands.defer(
                    () -> new ScoringSetpoints(elevator, wrist, ScoringSetpoint.BARGE),
                    scoringDependencies),
                () -> coral));

    operator
        .povLeft()
        .whileTrue(
            new ConditionalCommand(
                Commands.defer(
                    () -> new ScoringSetpoints(elevator, wrist, ScoringSetpoint.L3, true),
                    scoringDependencies),
                Commands.defer(
                    () -> new ScoringSetpoints(elevator, wrist, ScoringSetpoint.HIGH_ALGAE),
                    scoringDependencies),
                () -> coral));

    operator
        .povRight()
        .whileTrue(
            new ConditionalCommand(
                Commands.defer(
                    () -> new ScoringSetpoints(elevator, wrist, ScoringSetpoint.L2, true),
                    scoringDependencies),
                Commands.defer(
                    () -> new ScoringSetpoints(elevator, wrist, ScoringSetpoint.LOW_ALGAE),
                    scoringDependencies),
                () -> coral));

    operator
        .povDown()
        .whileTrue(
            Commands.defer(
                () -> new ScoringSetpoints(elevator, wrist, ScoringSetpoint.HOME),
                scoringDependencies));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
