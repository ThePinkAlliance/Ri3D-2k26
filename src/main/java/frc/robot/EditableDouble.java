package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.Supplier;

public class EditableDouble {
  private NetworkTableEntry entry;
  private double defaultValue;

  public EditableDouble(String name, double defaultValue) {
    this.entry = NetworkTableInstance.getDefault().getEntry(name);
    this.entry.setDefaultDouble(defaultValue);
    this.defaultValue = defaultValue;
  }

  public void setValue(double value) {
    this.entry.setDouble(value);
  }

  public void setDefaultValue(double value) {
    this.entry.setDefaultDouble(value);
    this.defaultValue = value;
  }

  public Supplier<Double> getValueSupplier() {
    return () -> this.entry.getDouble(defaultValue);
  }
}
