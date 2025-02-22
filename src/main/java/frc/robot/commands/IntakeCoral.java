// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.LightSensorSubsystem;
import frc.robot.subsystems.intake.WristIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private final WristIntakeSubsystem wristIntakeSubsystem;
  private final LightSensorSubsystem lightSensorSubsystem;
  private Voltage power;
  private boolean coralPhase1;
  private boolean done;
  private boolean init;
  private double startTime;


  public IntakeCoral(
      WristIntakeSubsystem wristIntakeSubsystem, LightSensorSubsystem lightSensorSubsystem) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;
    this.lightSensorSubsystem = lightSensorSubsystem;
    Timer timer = new Timer();
    Timer endTimer = new Timer();
    Timer timeoutTimer = new Timer();
    

    startTime = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristIntakeSubsystem.coast();
    
        wristIntakeSubsystem.updateBaselineCurrentDraw();
        coralPhase1 = false;
        done = false;
        endTimer.reset;
        thresholdTimer.reset();
        thresholdTimer.start();
        timeoutTimer.reset();
        init = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power
    wristIntakeSubsystem.setPower(0.85);
        
       
        if(wristIntakeSubsystem.noteDetectedByCurrent() && thresholdTimer.hasElapsed(1)){
            notePhaseOne = true;
            timeoutTimer.start();
        }
        if(notePhaseOne && !wristIntakeSubsystem.noteDetectedByCurrent()) {
            wristIntakeSubsystem.setSpeed(0);
            wristIntakeSubsystem.setSerializerSpeed(0.3);
            endTimer.start();
        }
        if(endTimer.hasElapsed(0.2)){
            done = true;
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristIntakeSubsystem.setPower(0);
    wristIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lightSensorSubsystem.hasCoral();
  }
}
