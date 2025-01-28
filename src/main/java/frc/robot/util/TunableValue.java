package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableValue {
  private double value;
  private String name;

  public TunableValue(String name, double defaultValue) {
    this.value = defaultValue;
    this.name = name;
    SmartDashboard.putNumber(name, defaultValue);
  }

  public double getValue() {
    this.value = SmartDashboard.getNumber(this.name, this.value);
    return this.value;
  }
}
