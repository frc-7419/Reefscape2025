// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.constants.Constants.ScoringConstants.ScoringPosition;
import frc.robot.constants.Constants.ScoringConstants.ScoringSetpoint;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.MaintainElevatorPosition;
import frc.robot.subsystems.intake.WristIntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndScore extends SequentialCommandGroup {
  /**
   * Creates a new AlignAndScore.
   *
   * <p>ONLY USE WHEN ALREADY AGAINST REEF WALL OR ELSE ROBOT WILL TIP AND BREAK AND THEY'RE GONNA
   * BLAME SOFTWARE
   */
  public AlignAndScore(
      CommandSwerveDrivetrain drivetrain,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      WristIntakeSubsystem wristIntake,
      ScoringPosition scoringPosition,
      ScoringSetpoint scoringSetpoint) {

    if (Robot.isReal()) {
      addCommands(
          new AlignToReef(drivetrain, scoringPosition, true).withTimeout(2),
          new ScoringSetpoints(elevator, wrist, scoringSetpoint),
          new ParallelDeadlineGroup(
              new RunCommand(() -> wristIntake.setPower(-0.5), wristIntake)
                  .until(() -> wristIntake.beamBreakisTriggered())
                  .finallyDo(() -> wristIntake.setPower(0)),
              new WristToPosition(wrist, Rotations.of(scoringSetpoint.wristAngle)),
              new MaintainElevatorPosition(elevator)),
          new MaintainElevatorPosition(elevator).withTimeout(0.1),
          new ScoringSetpoints(elevator, wrist, ScoringSetpoint.HOME)
              .until(() -> elevator.getPosition().lt(Rotations.of(1))));
    } else {
      addCommands(
          new AlignToReef(drivetrain, scoringPosition, true).withTimeout(2), new WaitCommand(2));
    }
  }
}
