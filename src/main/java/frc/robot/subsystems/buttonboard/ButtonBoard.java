package frc.robot.subsystems.buttonboard;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        SmartDashboard.putBoolean("Button 0 Pressed: ", buttonBoard.getRawButton(0));
        SmartDashboard.putBoolean("Button 1 Pressed: ", buttonBoard.getRawButton(1));
        SmartDashboard.putBoolean("Button 2 Pressed: ", buttonBoard.getRawButton(2));
        SmartDashboard.putBoolean("Button 3 Pressed: ", buttonBoard.getRawButton(3));
        SmartDashboard.putBoolean("Button 4 Pressed: ", buttonBoard.getRawButton(4));
        SmartDashboard.putBoolean("Button 5 Pressed: ", buttonBoard.getRawButton(5));
        SmartDashboard.putBoolean("Button 6 Pressed: ", buttonBoard.getRawButton(6));
        SmartDashboard.putBoolean("Button 7 Pressed: ", buttonBoard.getRawButton(7));
        SmartDashboard.putBoolean("Button 8 Pressed: ", buttonBoard.getRawButton(9));
        SmartDashboard.putBoolean("Button 9 Pressed: ", buttonBoard.getRawButton(9));
        SmartDashboard.putBoolean("Button 10 Pressed: ", buttonBoard.getRawButton(10));
        SmartDashboard.putBoolean("Button 11 Pressed: ", buttonBoard.getRawButton(11));
        SmartDashboard.putBoolean("Button 12 Pressed: ", buttonBoard.getRawButton(12));
        SmartDashboard.putBoolean("Button 13 Pressed: ", buttonBoard.getRawButton(13));
        SmartDashboard.putBoolean("Button 14 Pressed: ", buttonBoard.getRawButton(14));
        SmartDashboard.putBoolean("Button 15 Pressed: ", buttonBoard.getRawButton(15));
        SmartDashboard.putNumber("Left joystick X axis", controller.getLeftX());
        SmartDashboard.putNumber("Left joystick Y axis", controller.getLeftY());
        SmartDashboard.putNumber("Right joystick X axis", controller.getRightX());
        SmartDashboard.putNumber("Right joystick Y axis", controller.getRightY());
        
    }
}