// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.ScoringConstants.ScoringPosition;
import frc.robot.constants.Constants.ScoringConstants.ScoringSetpoint;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabCoral extends SequentialCommandGroup {
  /**
   * Creates a new GrabCoral.
   *
   * <p>This command first goes to the setpoint (HIGH_ALGAE or LOW_ALGAE based on what's stored in setpoint),
   * then does AlignToReef center slowly, and holds the scoring position the entire command.
   */
  public GrabCoral(
      CommandSwerveDrivetrain drivetrain,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      ScoringSetpoint setpoint) {

    addCommands(
        new ParallelCommandGroup(
            new AlignToReef(drivetrain, ScoringPosition.CENTER, true),
            new SequentialCommandGroup(
                new ScoringSetpoints(elevator, wrist, setpoint),
                new ParallelDeadlineGroup(
                    new WristToPosition(wrist, Rotations.of(setpoint.wristAngle)),
                    new MaintainElevatorPosition(elevator)),
                new MaintainElevatorPosition(elevator))));
  }
} 