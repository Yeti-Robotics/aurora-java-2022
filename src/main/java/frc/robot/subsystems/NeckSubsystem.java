package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;

public class NeckSubsystem extends SubsystemBase {

  private final CANSparkMax frontSpark;
  private final CANSparkMax rearSpark;
  private final DigitalInput lowerBeamBreak;
  private final DigitalInput upperBeamBreak;

  public NeckSubsystem() {
    frontSpark = new CANSparkMax(NeckConstants.FRONT_INDEXER, MotorType.kBrushless);
    rearSpark = new CANSparkMax(NeckConstants.REAR_INDEXER, MotorType.kBrushless);
    lowerBeamBreak = new DigitalInput(NeckConstants.NECK_LOWER_BEAM_BREAK);
    upperBeamBreak = new DigitalInput(NeckConstants.NECK_UPPER_BEAM_BREAK);
  }

  @Override
  public void periodic() {}

  public void moveUp() {
    frontSpark.set(NeckConstants.NECK_FRONT_SPEED);
    rearSpark.set(NeckConstants.NECK_REAR_SPEED);
  }

  public void moveUp(double speed) {
    frontSpark.set(speed);
    rearSpark.set(speed);
  }

  public void moveFrontUp() {
    frontSpark.set(NeckConstants.NECK_FRONT_SPEED);
  }

  public void moveRearUp() {
    rearSpark.set(NeckConstants.NECK_REAR_SPEED);
  }

  public void moveDown() {
    frontSpark.set(-NeckConstants.NECK_FRONT_OUT_SPEED);
    rearSpark.set(-NeckConstants.NECK_REAR_OUT_SPEED);
  }

  public void stopNeck() {
    frontSpark.set(0.0);
    rearSpark.set(0.0);
  }

  public boolean getLowerBeamBreak() {
    return lowerBeamBreak.get();
  }

  public boolean getUpperBeamBreak() {
    return upperBeamBreak.get();
  }
}
