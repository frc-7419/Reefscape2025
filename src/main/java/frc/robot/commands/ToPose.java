package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class ToPose extends SequentialCommandGroup {
    private Pose2d targetPose;

    public ToPose(CommandSwerveDrivetrain drivetrain) {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        SmartDashboard.putBoolean("alliance present", alliance.isPresent());
        targetPose = new Pose2d(3.812, 5.392, Rotation2d.fromDegrees(-20));

        PathConstraints pathConstraints = new PathConstraints(
                3, 4, Units.degreesToRadians(540), Units.degreesToRadians(720));
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, pathConstraints, 0.0);

        addCommands(pathfindingCommand);
    }
}