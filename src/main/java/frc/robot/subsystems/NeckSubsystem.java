package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;

public class NeckSubsystem extends SubsystemBase {
    private CANSparkMax neckFrontSpark;
    private CANSparkMax neckBackSpark;
    private DigitalInput lowerBeamBreak;
    private DigitalInput upperBeamBreak;

    public NeckSubsystem() {
        neckFrontSpark = new CANSparkMax(NeckConstants.NECK_MOTOR_FRONT, MotorType.kBrushless);
        neckBackSpark = new CANSparkMax(NeckConstants.NECK_MOTOR_BACK, MotorType.kBrushless);
        lowerBeamBreak = new DigitalInput(NeckConstants.NECK_LOWER_BEAM_BREAK);
        upperBeamBreak = new DigitalInput(NeckConstants.NECK_UPPER_BEAM_BREAK);
    }

    public void moveUp(){
        neckFrontSpark.set(NeckConstants.NECK_SPEED);
        neckBackSpark.set(NeckConstants.NECK_SPEED);
    }

    public void moveDown(){
        neckFrontSpark.set(-NeckConstants.NECK_SPEED);
        neckBackSpark.set(-NeckConstants.NECK_SPEED);
    }

    public void stopNeck(){
        neckFrontSpark.set(0.0);
        neckBackSpark.set(0.0);
    }

    public boolean getLowerBeamBreak(){
        return lowerBeamBreak.get();
    }
    public boolean getUpperBeamBreak(){
        return upperBeamBreak.get();
    }
}
