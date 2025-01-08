package frc.robot.subsystems.buttonboard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ButtonBoard extends SubsystemBase {
  private final GenericHID buttonBoard;
  private final CommandXboxController controller;

  public ButtonBoard(GenericHID buttonBoard, CommandXboxController controller) {
    this.buttonBoard = buttonBoard;
    this.controller = controller;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Button States: ", "0 for not pressed 1 for pressed");
    for (int i = 0; i < 12; i++) {
      SmartDashboard.putBoolean("Button " + i + " Pressed: ", buttonBoard.getRawButton(i));
    }
    SmartDashboard.putNumber("Left joystick X axis", controller.getLeftX());
    SmartDashboard.putNumber("Left joystick Y axis", controller.getLeftY());
    SmartDashboard.putNumber("Right joystick X axis", controller.getRightX());
    SmartDashboard.putNumber("Right joystick Y axis", controller.getRightY());
  }
}

