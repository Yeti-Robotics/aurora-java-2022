package frc.robot.di;

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.*;
import javax.inject.Named;
import javax.inject.Singleton;

@Module
public class SubsystemsModule {
  @Singleton
  @Provides
  @Named("led")
  public Subsystem provideLedSubsystem() {
    return new LEDSubsystem();
  }

  @Singleton
  @Provides
  @Named("intake")
  public Subsystem provideIntakeSubsystem() {
    return new IntakeSubsystem();
  }

  @Singleton
  @Provides
  @Named("shifting")
  public Subsystem provideShiftingSubsystem() {
    return new ShiftingSubsystem();
  }

  @Singleton
  @Provides
  @Named("neck")
  public Subsystem provideNeckSubsystem() {
    return new NeckSubsystem();
  }

  @Singleton
  @Provides
  @Named("turret")
  public Subsystem provideTurretSubsystem() {
    return new TurretSubsystem();
  }

  @Singleton
  @Provides
  @Named("shooter")
  public Subsystem provideShooterSubsystem() {
    return new ShooterSubsystem();
  }

  @Singleton
  @Provides
  @Named("climber")
  public Subsystem provideClimberSubsystem() {
    return new ClimberSubsystem();
  }

  @Singleton
  @Provides
  @Named("drivetrain")
  public Subsystem provideDrivetrainSubsystem(
      @Named("tank drive") Command tankCommand,
      @Named("cheezy drive") Command cheezyCommand,
      @Named("arcade drive") Command arcadeCommand) {
    return new DrivetrainSubsystem(tankCommand, cheezyCommand, arcadeCommand);
  }
}
