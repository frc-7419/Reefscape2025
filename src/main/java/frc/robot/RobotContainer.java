// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RunHanger;

import frc.robot.subsystems.hangersubsystem;

public class RobotContainer {
  private final hangersubsystem hanger = new hangersubsystem();
  private final RunHanger runHanger = new RunHanger();



  private final CommandXboxController driver = new CommandXboxController(1);
  private final CommandXboxController operator = new CommandXboxController(0);

  

  public RobotContainer() {
    

    configureBindings();
    

    // antiTip.schedule();
  }


  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    
     
    operator.a().whileTrue(runHanger);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return new PathPlannerAuto("Example Path");
  }
}
