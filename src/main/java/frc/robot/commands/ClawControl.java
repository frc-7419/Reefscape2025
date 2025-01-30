// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Timer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ClawConstants;
import frc.robot.subsystems.claw.ClawSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawControl extends Command {
  /** Creates a new OpenClaw. */
  private ClawSubsystem clawSubsystem;

  private PIDController openingPid = new PIDController(1, 0, 1); // need values later
  private PIDController closingPid = new PIDController(1, 0, 1); // need values later

  public ClawControl(
      ClawSubsystem clawSubsystem,
      Angle openingSetpoint,
      Angle closingSetpoint) { // TODO: figure out the value of setpoint for the desired claw open
    this.clawSubsystem = clawSubsystem;
    this.openingPid = new PIDController(1, 0, 0.5);
    this.closingPid = new PIDController(1, 0, 0.5);
    //openingPid.setSetpoint(openingSetpoint);
    //closingPid.setSetpoint(closingSetpoint);
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    while (clawSubsystem.getBeamBreak()) {
      //Thread.sleep(100);, a delay so the claw doesnt close prematurely, I dont know how to fix the error.
      clawSubsystem.setPosition(ClawConstants.kClawCloseSetpoint);
    }
    while (!(clawSubsystem.getBeamBreak())) {
      //Thread.sleep(100);
      clawSubsystem.setPosition(ClawConstants.kClawOpenSetpoint);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
