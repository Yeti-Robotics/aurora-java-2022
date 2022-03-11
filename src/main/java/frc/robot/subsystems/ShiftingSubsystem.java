package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShiftingSubsystem extends SubsystemBase {
    private DoubleSolenoid shifter;

    public enum ShiftStatus {
        HIGH, LOW
    }

    public static ShiftStatus shiftStatus;

    public ShiftingSubsystem() {
        shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveConstants.SOLENOID_SHIFTER[0], DriveConstants.SOLENOID_SHIFTER[1]);
        shiftUp();
    }

    public void shiftUp() {
        shifter.set(DoubleSolenoid.Value.kForward);
        shiftStatus = ShiftStatus.HIGH;
    }

    public void shiftDown() {
        shifter.set(DoubleSolenoid.Value.kReverse);
        shiftStatus = ShiftStatus.LOW;
    }

    public static ShiftStatus getShifterPosition() {
        return shiftStatus;
    }
}
