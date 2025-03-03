// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignAndScore;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ToPose;
import frc.robot.constants.Constants.CameraConfig;
import frc.robot.constants.Constants.DrivetrainConstants;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.ScoringConstants.ScoringPosition;
import frc.robot.constants.Constants.ScoringConstants.ScoringSetpoint;
import frc.robot.constants.Constants.VisionConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorMM;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
import frc.robot.subsystems.elevator.RunElevatorWithJoystick;
import frc.robot.subsystems.elevator.RunElevatorWithPID;
import frc.robot.subsystems.intake.IntakeWithBeamBreak;
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
import java.util.function.DoubleSupplier;

public class RobotContainer {
        private double MaxSpeed = DrivetrainConstants.kMaxVelocity.in(MetersPerSecond);
        private double MaxAngularRate = DrivetrainConstants.kMaxAngularRate.in(RotationsPerSecond);

        private WristIntakeSubsystem wristIntakeSubsystem = new WristIntakeSubsystem();

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        private RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(wristIntakeSubsystem, operator);
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.05)
                        .withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
                        .withDriveRequestType(
                                        DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final ToPose toPose = new ToPose(drivetrain);

        private WristSubsystem wrist = new WristSubsystem();
        private final ElevatorSubsystem elevator = new ElevatorSubsystem(wrist::getPosition);

    // private final WristSubsystem wrist = new WristSubsystem();
    public final VisionSubsystem photonvision;
    private final CameraConfig photonCamOne = new CameraConfig("Cam1", VisionConstants.kRobotToCamOne);
    private final CameraConfig photonCamTwo = new CameraConfig("Cam2", VisionConstants.kRobotToCamOne);

        private final List<CameraConfig> cameraConfigs = new ArrayList<CameraConfig>() {
                {
                        add(photonCamOne);
                        add(photonCamTwo);
                }
        };

        private boolean coral = true;

        public RobotContainer() {
                photonvision = new VisionSubsystem(cameraConfigs);

                configureBindings();
                SmartDashboard.putBoolean("isConfigured", AutoBuilder.isConfigured());

                SmartDashboard.putBoolean("Coral Mode", coral);
                registerNamedCommands();
        }

        private final Command alignAndScoreL1Left = new AlignAndScore(drivetrain, elevator, wrist, wristIntakeSubsystem,
                        ScoringPosition.LEFT, ScoringSetpoint.L1);
        private final Command alignAndScoreL1Right = new AlignAndScore(drivetrain, elevator, wrist,
                        wristIntakeSubsystem,
                        ScoringPosition.RIGHT, ScoringSetpoint.L1);

        private final Command alignAndScoreL2Left = new AlignAndScore(drivetrain, elevator, wrist, wristIntakeSubsystem,
                        ScoringPosition.LEFT, ScoringSetpoint.L2);
        private final Command alignAndScoreL2Right = new AlignAndScore(drivetrain, elevator, wrist,
                        wristIntakeSubsystem,
                        ScoringPosition.RIGHT, ScoringSetpoint.L2);

        private final Command alignAndScoreL3Left = new AlignAndScore(drivetrain, elevator, wrist, wristIntakeSubsystem,
                        ScoringPosition.LEFT, ScoringSetpoint.L3);
        private final Command alignAndScoreL3Right = new AlignAndScore(drivetrain, elevator, wrist,
                        wristIntakeSubsystem,
                        ScoringPosition.RIGHT, ScoringSetpoint.L3);

        private final Command alignAndScoreL4Left = new AlignAndScore(drivetrain, elevator, wrist, wristIntakeSubsystem,
                        ScoringPosition.LEFT, ScoringSetpoint.L4);
        private final Command alignAndScoreL4Right = new AlignAndScore(drivetrain, elevator, wrist,
                        wristIntakeSubsystem,
                        ScoringPosition.RIGHT, ScoringSetpoint.L4);

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

                NamedCommands.registerCommands(namedCommands);
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(
                                                () -> drive
                                                                .withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive
                                                                                                              // forward
                                                                                                              // with
                                                                // negative Y (forward)
                                                                .withVelocityY(
                                                                                -driver.getLeftX() * MaxSpeed) // Drive
                                                                                                               // left
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                                                                                               // (left)
                                                                .withRotationalRate(
                                                                                -driver.getRightX()
                                                                                                * MaxAngularRate) // Drive
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
                                                                () -> point.withModuleDirection(
                                                                                new Rotation2d(-driver.getLeftY(),
                                                                                                -driver.getLeftX()))));

                driver.leftTrigger(0.2).whileTrue(new AlignToReef(drivetrain, ScoringPosition.LEFT));
                driver.rightTrigger(0.2).whileTrue(new AlignToReef(drivetrain, ScoringPosition.RIGHT));

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
                operator.start().onTrue(new RunCommand(() -> elevator.zeroEncoder(), elevator));
                operator.leftBumper().whileTrue(new IntakeWithBeamBreak(wristIntakeSubsystem));
                // operator.y().whileTrue(new WristPIDTest(wristSubsystem));
                operator.a().whileTrue(new WristToPosition(wrist, Rotations.of(0.118
                )));
                operator.b().whileTrue(new WristToPosition(wrist, Rotations.of(0.34)));

                wristIntakeSubsystem.setDefaultCommand(runIntakeWithJoystick);
                wrist.setDefaultCommand(new RunWristWithJoystick(wrist, () -> operator.getRightY() * 0.3));

                elevator.setDefaultCommand(new MaintainElevatorPosition(elevator));
                DoubleSupplier elevatorPowerSupplier = () -> operator.getLeftY();
                new Trigger(
                                () -> (Math.abs(elevatorPowerSupplier
                                                .getAsDouble()) > ElevatorConstants.joystickDeadband))
                                .whileTrue(
                                                new RunElevatorWithJoystick(elevator, elevatorPowerSupplier,
                                                                wrist::getPosition));

                wristIntakeSubsystem.isHolding()
                                .negate()
                                .whileTrue(new RunIntakeWithJoystick(wristIntakeSubsystem, operator));
                operator.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.01)
                                .and(wristIntakeSubsystem.isHolding())
                                .onTrue(Commands.run(() -> wristIntakeSubsystem.setHolding(false),
                                                wristIntakeSubsystem));
                wristIntakeSubsystem.isHolding()
                                .onTrue(
                                                Commands.runEnd(
                                                                () -> wristIntakeSubsystem.setTorque(Amps.of(10)),
                                                                () -> wristIntakeSubsystem.setTorque(Amps.of(0)),
                                                                wristIntakeSubsystem));

                // L1: 0
                // L2:
                // L3:
                // L4:
                
                operator.start().onTrue(new InstantCommand(() -> {
                        coral = !coral;
                        SmartDashboard.putBoolean("Coral Mode", coral);
                }));

                // operator.povUp().whileTrue(
                //                 new ConditionalCommand(
                //                                 new ScoringSetpoints(elevator, wrist, ScoringSetpoint.L4),
                //                                 new ScoringSetpoints(elevator, wrist, ScoringSetpoint.BARGE),
                //                                 () -> coral));

                // operator.povLeft().whileTrue(
                //                 new ConditionalCommand(
                //                                 new ScoringSetpoints(elevator, wrist, ScoringSetpoint.L3),
                //                                 new ScoringSetpoints(elevator, wrist, ScoringSetpoint.HIGH_ALGAE),
                //                                 () -> coral));

                // operator.povRight().whileTrue(
                //                 new ConditionalCommand(
                //                                 new ScoringSetpoints(elevator, wrist, ScoringSetpoint.L2),
                //                                 new ScoringSetpoints(elevator, wrist, ScoringSetpoint.LOW_ALGAE),
                //                                 () -> coral));

                // operator.povDown().whileTrue(new ScoringSetpoints(elevator, wrist, ScoringSetpoint.HOME));
                operator.povUp().whileTrue(
                                Commands.defer(() -> {
                                        // if (coral)
                                                return setpointCommand(ScoringSetpoint.L3);
                                        // else
                                        //         return setpointCommand(ScoringSetpoint.BARGE);
                                }, new HashSet<>(Arrays.asList(elevator, wrist))));
                                
        }

        public Command setpointCommand(ScoringSetpoint targetPosition) {
                SequentialCommandGroup commandGroup = new SequentialCommandGroup();
                double elevatorRotations = elevator.getPosition().in(Rotations);
                Angle upAngle = targetPosition.name.equals("BARGE") ? Rotations.of(0.04) : Rotations.of(0.118);

        
                commandGroup.addCommands(
                        Commands.parallel(
                                new ElevatorMM(elevator, ElevatorConstants.kElevatorBarLowerLimit),
                                new WristToPosition(wrist, upAngle)
                        ),
                        Commands.parallel(
                                new ElevatorMM(elevator, Rotations.of(targetPosition.elevatorHeight)),
                                new WristToPosition(wrist, Rotations.of(targetPosition.wristAngle))
                        )
                );

                return(commandGroup);
        }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return new PathPlannerAuto("Example Path");
    }
}
