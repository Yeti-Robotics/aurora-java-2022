package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;

public class NeckSubsystem extends SubsystemBase {
    private CANSparkMax FRONT_INDEXER;
    private CANSparkMax REAR_INDEXER;
    private DigitalInput lowerBeamBreak;
    private DigitalInput upperBeamBreak;

    public NeckSubsystem() {
        FRONT_INDEXER = new CANSparkMax(NeckConstants.FRONT_INDEXER, MotorType.kBrushless);
        REAR_INDEXER = new CANSparkMax(NeckConstants.REAR_INDEXER, MotorType.kBrushless);
        lowerBeamBreak = new DigitalInput(NeckConstants.NECK_LOWER_BEAM_BREAK);
        upperBeamBreak = new DigitalInput(NeckConstants.NECK_UPPER_BEAM_BREAK);
    }

    public void moveUp(){
        FRONT_INDEXER.set(NeckConstants.NECK_SPEED);
        REAR_INDEXER.set(NeckConstants.NECK_SPEED);
    }

    public void moveDown(){
        FRONT_INDEXER.set(-NeckConstants.NECK_SPEED);
        REAR_INDEXER.set(-NeckConstants.NECK_SPEED);
    }

    public void stopNeck(){
        FRONT_INDEXER.set(0.0);
        REAR_INDEXER.set(0.0);
    }

    public boolean getLowerBeamBreak(){
        return lowerBeamBreak.get();
    }
    public boolean getUpperBeamBreak(){
        return upperBeamBreak.get();
    }
}
