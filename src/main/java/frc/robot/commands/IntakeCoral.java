// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.LightSensorSubsystem;
import frc.robot.subsystems.intake.WristIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private final WristIntakeSubsystem wristIntakeSubsystem;

  private boolean coralPhase1;

  private double startTime;
  private final Timer thresholdTimer;
  private final Timer timeoutTimer;
  private final Timer endTimer;
  private boolean done = false;

  public IntakeCoral(
      WristIntakeSubsystem wristIntakeSubsystem) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;
    this.timeoutTimer = new Timer();
    this.thresholdTimer = new Timer();
    this.endTimer = new Timer();

    startTime = Timer.getFPGATimestamp();

    addRequirements(wristIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    wristIntakeSubsystem.setVoltage(-2);

    coralPhase1 = false;

    endTimer.reset();
    thresholdTimer.reset();
    thresholdTimer.start();
    timeoutTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Coral Phase 1", coralPhase1);

    if (wristIntakeSubsystem.coralDetectedByCurrent() && thresholdTimer.hasElapsed(0.05)) {
      coralPhase1 = true;
      endTimer.start();
    }
    if (endTimer.hasElapsed(0.5)) {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    wristIntakeSubsystem.setVoltage(0);

    wristIntakeSubsystem.brake();
    thresholdTimer.stop();
    timeoutTimer.stop();
    endTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}