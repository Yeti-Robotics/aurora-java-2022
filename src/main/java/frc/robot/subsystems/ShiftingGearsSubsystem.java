package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShiftingGearsSubsystem extends SubsystemBase {
    private DoubleSolenoid shifter;
    public enum ShiftStatus{
        HIGH, LOW
    }
    public static ShiftStatus shiftStatus;

    public ShiftingGearsSubsystem() {
        shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.SOLENOID_SHIFTER[0], DriveConstants.SOLENOID_SHIFTER[1]);
        shiftStatus = ShiftStatus.LOW;
    }

    public void shiftUp() {
        shifter.set(DoubleSolenoid.Value.kForward);
        shiftStatus = ShiftStatus.HIGH;
    }

    //Shifts the drive train into low gear
    public void shiftDown() {
        shifter.set(DoubleSolenoid.Value.kReverse);
        shiftStatus = ShiftStatus.LOW;
    }



    //Finds out what position the shifter is currently in
    public static ShiftStatus getShifterPosition() {
        return shiftStatus;
    }

}