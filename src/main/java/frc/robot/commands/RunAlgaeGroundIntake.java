// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.algae.AlgaeIntakeSubsystem;

// /* You should consider using the more terse Command factories API instead
// https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class RunAlgaeGroundIntake extends Command {
//   /** Creates a new RunAlgaeGroundIntake. */
//   private AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem(TalonFX motor1,
// TalonFX motor2);
//   private XboxController joystick;
//   public RunAlgaeGroundIntake(AlgaeIntakeSubsystem algaeIntakeSubsystem, XboxController joystick)
//   {
//     joystick = new XboxController(0);
//     this.algaeIntakeSubsystem = algaeIntakeSubsystem;
//     this.joystick = joystick;
//     addRequirements(algaeIntakeSubsystem);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     algaeIntakeSubsystem.coast();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     algaeIntakeSubsystem.set(joystick.getLeftY() * 0.5f);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     algaeIntakeSubsystem.brake();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
