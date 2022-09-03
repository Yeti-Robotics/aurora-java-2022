package frc.robot.di;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import dagger.Module;
import dagger.Provides;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.*;
import javax.inject.Named;
import javax.inject.Singleton;

@Module
public class SubsystemsModule {
  @Singleton
  @Provides
  @Named("led")
  public Subsystem provideLedSubsystem(AddressableLED ledStrip, AddressableLEDBuffer ledBuffer) {
    return new LEDSubsystem(ledStrip, ledBuffer);
  }

  @Singleton
  @Provides
  @Named("intake")
  public Subsystem provideIntakeSubsystem(
      @Named("intake pistons") DoubleSolenoid intakePistons,
      @Named("intake falcon") TalonFX intakeFalcon) {
    return new IntakeSubsystem(intakePistons, intakeFalcon);
  }

  @Singleton
  @Provides
  @Named("shifting")
  public Subsystem provideShiftingSubsystem(@Named("shifter") DoubleSolenoid shifter) {
    return new ShiftingSubsystem(shifter);
  }

  @Singleton
  @Provides
  @Named("neck")
  public Subsystem provideNeckSubsystem(
      @Named("front spark") CANSparkMax frontSpark,
      @Named("rear spark") CANSparkMax rearSpark,
      @Named("lower beam break") DigitalInput lowerBeamBreak,
      @Named("upper beam break") DigitalInput upperBeamBreak) {
    return new NeckSubsystem(frontSpark, rearSpark, lowerBeamBreak, upperBeamBreak);
  }

  @Singleton
  @Provides
  @Named("turret")
  public Subsystem provideTurretSubsystem(
      @Named("turret spark") CANSparkMax turret, @Named("mag switch") DigitalInput magSwitch) {
    return new TurretSubsystem(turret, magSwitch);
  }

  @Singleton
  @Provides
  @Named("shooter")
  public Subsystem provideShooterSubsystem(
      @Named("shooter left") WPI_TalonFX shooterLeft,
      @Named("shooter right") WPI_TalonFX shooterRight,
      @Named("shooter pid") PIDController pidController,
      @Named("feed forward") SimpleMotorFeedforward feedForward) {
    return new ShooterSubsystem(shooterLeft, shooterRight, pidController, feedForward);
  }

  @Singleton
  @Provides
  @Named("climber")
  public Subsystem provideClimberSubsystem(
      @Named("climber 1") WPI_TalonFX climber1,
      @Named("climber 2") WPI_TalonFX climber2,
      @Named("climber brake") DoubleSolenoid climberBrake) {
    return new ClimberSubsystem(climber1, climber2, climberBrake);
  }

  @Singleton
  @Provides
  @Named("drivetrain")
  public Subsystem provideDrivetrainSubsystem(
      RobotComponent robotComponent,
      @Named("left drive 1") WPI_TalonFX leftDrive1,
      @Named("left drive 2") WPI_TalonFX leftDrive2,
      @Named("right drive 1") WPI_TalonFX rightDrive1,
      @Named("right drive 2") WPI_TalonFX rightDrive2,
      AHRS gyro,
      @Named("drive PID") PIDController drivePID,
      DifferentialDriveWheelSpeeds diffWheelSpeeds) {
    return new DrivetrainSubsystem(
        robotComponent,
        leftDrive1,
        leftDrive2,
        rightDrive1,
        rightDrive2,
        gyro,
        drivePID,
        diffWheelSpeeds);
  }
}
