// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants.RobotConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionResult;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.CombinedAlert;
import frc.robot.util.TunableBoolean;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  Timer gcTimer = new Timer();

  private final Field2d vision = new Field2d();

  private final TunableBoolean useVision = new TunableBoolean("Use Vision", true);

  private final CombinedAlert canErrorAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "RIO CAN errors detected.",
          "The RIO CAN bus is not responding properly.");
  private final CombinedAlert canivoreErrorAlert =
      new CombinedAlert(
          CombinedAlert.Severity.ERROR,
          "CANivore error detected.",
          "The CANivore device is not responding properly.");
  private final CombinedAlert lowBatteryAlert =
      new CombinedAlert(
          CombinedAlert.Severity.WARNING,
          "Low Battery / Brownout",
          "Voltage is under " + RobotConstants.kLowBatteryVoltage + "V.");
  CANBus canivore = new CANBus(RobotConstants.kCANivoreBus);

  private void updateRobotStatus() {
    CANStatus rioCanStatus = RobotController.getCANStatus();
    CANBusStatus canivoreStatus = canivore.getStatus();

    SmartDashboard.putNumber(
        "RIO CAN Bus Utilization (%)", rioCanStatus.percentBusUtilization * 100);
    SmartDashboard.putNumber("RIO CAN Bus Off Count", rioCanStatus.busOffCount);
    SmartDashboard.putNumber("RIO CAN TX Full Count", rioCanStatus.txFullCount);
    SmartDashboard.putNumber("RIO CAN Receive Error Count", rioCanStatus.receiveErrorCount);
    SmartDashboard.putNumber("RIO CAN Transmit Error Count", rioCanStatus.transmitErrorCount);

    SmartDashboard.putNumber(
        "CANivore CAN Bus Utilization (%)", canivoreStatus.BusUtilization * 100);
    SmartDashboard.putNumber("CANivore CAN Bus Off Count", canivoreStatus.BusOffCount);
    SmartDashboard.putNumber("CANivore CAN TX Full Count", canivoreStatus.TxFullCount);
    SmartDashboard.putString("CANivore CAN Status", canivoreStatus.Status.getName());

    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());

    if (rioCanStatus.receiveErrorCount > 0 || rioCanStatus.transmitErrorCount > 0) {
      canErrorAlert.set(true);
    } else {
      canErrorAlert.set(false);
    }
    if (canivoreStatus.Status.isError()) {
      canivoreErrorAlert.set(true);
    } else {
      canivoreErrorAlert.set(false);
    }
    if (RobotController.getBatteryVoltage() <= RobotConstants.kLowBatteryVoltage) {
      lowBatteryAlert.set(true);
    } else {
      lowBatteryAlert.set(false);
    }
  }

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {

    if (gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }

    CommandScheduler.getInstance().run();

    if (useVision.getValue()) {
      VisionSubsystem vision = m_robotContainer.photonvision;
      CommandSwerveDrivetrain drivetrain = m_robotContainer.drivetrain;

      for (VisionResult result : vision.getIndividualVisionEstimates()) {
        Pose2d pose = result.estimatedRobotPose.estimatedPose.toPose2d();

        drivetrain.addVisionMeasurement(
            pose,
            Utils.fpgaToCurrentTime(result.estimatedRobotPose.timestampSeconds),
            result.stdDevs);

        SmartDashboard.putNumberArray(
            "Vision Pose " + result.cameraName,
            new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
      }

      if (isSimulation()) {
        vision.simulationPeriodic(drivetrain.getState().Pose);
      }
    }

    updateRobotStatus();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
