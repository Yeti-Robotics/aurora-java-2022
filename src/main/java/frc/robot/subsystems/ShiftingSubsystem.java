package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShiftingSubsystem extends SubsystemBase {

  private final DoubleSolenoid shifter;

  public enum ShiftStatus {
    HIGH,
    LOW
  }

  public static ShiftStatus shiftStatus;

  public ShiftingSubsystem(DoubleSolenoid shifterSolonoid) {
    shifter = shifterSolonoid;
    shiftUp();
  }

  public void shiftUp() {
    shifter.set(DoubleSolenoid.Value.kReverse);
    shiftStatus = ShiftStatus.HIGH;
  }

  public void shiftDown() {
    shifter.set(DoubleSolenoid.Value.kForward);
    shiftStatus = ShiftStatus.LOW;
  }

  public static ShiftStatus getShifterPosition() {
    return shiftStatus;
  }
}
