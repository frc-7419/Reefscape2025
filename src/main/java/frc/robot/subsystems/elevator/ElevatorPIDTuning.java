// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPIDTuning extends Command {
  ElevatorSubsystem elevatorSubsystem;
  CommandXboxController joystick;
  double voltage = 0;
  /** Creates a new ElevatorPIDTuning. */
  public ElevatorPIDTuning(ElevatorSubsystem elevatorSubsystem, CommandXboxController joystick) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
   
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.leftBumper().getAsBoolean()){
      voltage-= 0.001;
    } else if(joystick.rightBumper().getAsBoolean()){
      voltage+= 0.001;
    }
    voltage = Math.max(-3, Math.min(3, voltage));
    System.out.println(voltage);
    elevatorSubsystem.setVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    elevatorSubsystem.setVoltage(voltage);
    elevatorSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
