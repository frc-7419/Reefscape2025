package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableBoolean {
  private boolean value;
  private String name;

  public TunableBoolean(String name, boolean defaultValue) {
    this.value = defaultValue;
    this.name = name;
    SmartDashboard.putBoolean(name, defaultValue);
  }

  public boolean getValue() {
    this.value = SmartDashboard.getBoolean(this.name, this.value);
    return this.value;
  }
}
