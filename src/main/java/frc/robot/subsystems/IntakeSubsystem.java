package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import javax.inject.Named;

public class IntakeSubsystem extends SubsystemBase implements AutoCloseable {

  public enum IntakeStatus {
    OUT,
    IN
  }

  private IntakeStatus intakeStatus;

  private final TalonFX intakeFalcon;
  private final DoubleSolenoid pistons;

  public IntakeSubsystem(
      @Named("intake pistons") DoubleSolenoid intakePistons,
      @Named("intake falcon") WPI_TalonFX intakeFalcon) {
    pistons = intakePistons;
    this.intakeFalcon = intakeFalcon;
    intakeStatus = IntakeStatus.IN;
  }

  @Override
  public void close() throws Exception {
    pistons.close();
  }

  public void extend() {
    pistons.set(DoubleSolenoid.Value.kReverse);
    intakeStatus = IntakeStatus.OUT;
  }

  public void retract() {
    pistons.set(DoubleSolenoid.Value.kForward);
    intakeStatus = IntakeStatus.IN;
  }

  public void rollIn() {
    intakeFalcon.set(ControlMode.PercentOutput, IntakeConstants.INTAKE_SPEED);
  }

  public void rollIn(double speed) {
    intakeFalcon.set(ControlMode.PercentOutput, speed);
  }

  public void rollOut() {
    intakeFalcon.set(ControlMode.PercentOutput, -IntakeConstants.INTAKE_OUT_SPEED);
  }

  public void stopRoll() {
    intakeFalcon.set(ControlMode.PercentOutput, 0.0);
  }

  public void toggleIntake() {
    pistons.toggle();
  }

  public IntakeStatus getIntakePostion() {
    return pistons.get() == DoubleSolenoid.Value.kForward ? IntakeStatus.IN : IntakeStatus.OUT;
  }

  public IntakeStatus getIntakePositionDown() {
    return IntakeStatus.OUT;
  }

  public IntakeStatus getIntakePositionUp() {
    return IntakeStatus.IN;
  }
}
