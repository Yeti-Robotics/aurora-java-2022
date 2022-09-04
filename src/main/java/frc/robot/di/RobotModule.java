package frc.robot.di;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import dagger.Module;
import dagger.Provides;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.LED.ShooterLEDCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.AutoHelper;
import javax.inject.Named;
import javax.inject.Singleton;

@Module
public class RobotModule {
  @Singleton
  @Provides
  public RobotContainer provideRobotContainer(
      RobotComponent robotComponent,
      Joystick driverJoystick,
      @Named("climber") Subsystem climberSubsystem,
      @Named("drivetrain") Subsystem drivetrainSubsystem,
      @Named("intake") Subsystem intakeSubsystem,
      @Named("led") Subsystem ledSubsystem,
      @Named("neck") Subsystem neckSubsystem,
      @Named("shifting") Subsystem shiftingSubsystem,
      @Named("shooter") Subsystem shooterSubsystem,
      @Named("turret") Subsystem turretSubsystem,
      ShooterLEDCommand shooterLEDCommand,
      ToggleIntakeCommand toggleIntakeCommand,
      @Named("two ball") Command twoBall,
      @Named("two ball alt") Command twoBallAlt,
      @Named("one ball") Command oneBall,
      @Named("four ball") Command fourBall,
      @Named("three ball") Command threeBall,
      @Named("two ball dump") Command twoBallDump,
      @Named("test auto") Command testAuto) {
    return new RobotContainer(
        robotComponent,
        driverJoystick,
        (ClimberSubsystem) climberSubsystem,
        (DrivetrainSubsystem) drivetrainSubsystem,
        (IntakeSubsystem) intakeSubsystem,
        (LEDSubsystem) ledSubsystem,
        (NeckSubsystem) neckSubsystem,
        (ShiftingSubsystem) shiftingSubsystem,
        (ShooterSubsystem) shooterSubsystem,
        (TurretSubsystem) turretSubsystem,
        shooterLEDCommand,
        toggleIntakeCommand,
        twoBall,
        twoBallAlt,
        oneBall,
        fourBall,
        threeBall,
        twoBallDump,
        testAuto);
  }

  /****** Robot Class Dependencies ******/

  @Provides
  public PowerDistribution provideRevPDH() {
    return new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
  }

  @Provides
  public Joystick provideJoystick() {
    return new Joystick(Constants.OIConstants.DRIVER_STATION_JOY);
  }

  /****** Climber Subsystem Dependencies ******/

  @Provides
  @Named("climber 1")
  public WPI_TalonFX provideClimberFalcon1() {
    return new WPI_TalonFX(Constants.ClimberConstants.CLIMBER_1);
  }

  @Provides
  @Named("climber 2")
  public WPI_TalonFX provideClimberFalcon2() {
    return new WPI_TalonFX(Constants.ClimberConstants.CLIMBER_2);
  }

  @Provides
  @Named("climber brake")
  public DoubleSolenoid provideClimberBrake() {
    return new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        Constants.ClimberConstants.CLIMBER_BRAKE[0],
        Constants.ClimberConstants.CLIMBER_BRAKE[1]);
  }

  /****** Drivetrain Subsystem Dependencies ******/

  @Provides
  @Named("left drive 1")
  public WPI_TalonFX provideLeftDriveFalcon1() {
    WPI_TalonFX drive = new WPI_TalonFX(Constants.DriveConstants.LEFT_FALCON_1);
    drive.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    drive.enableVoltageCompensation(true);
    drive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    return drive;
  }

  @Provides
  @Named("left drive 2")
  public WPI_TalonFX provideLeftDriveFalcon2() {
    WPI_TalonFX drive = new WPI_TalonFX(Constants.DriveConstants.LEFT_FALCON_2);
    drive.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    drive.enableVoltageCompensation(true);

    return drive;
  }

  @Provides
  @Named("right drive 1")
  public WPI_TalonFX provideRightDriveFalcon1() {
    WPI_TalonFX drive = new WPI_TalonFX(Constants.DriveConstants.RIGHT_FALCON_1);
    drive.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    drive.enableVoltageCompensation(true);
    drive.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    return drive;
  }

  @Provides
  @Named("right drive 2")
  public WPI_TalonFX provideRightDriveFalcon2() {
    WPI_TalonFX drive = new WPI_TalonFX(Constants.DriveConstants.RIGHT_FALCON_2);
    drive.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    drive.enableVoltageCompensation(true);

    return drive;
  }

  @Provides
  public AHRS provideDriveGyro() {
    return new AHRS(SerialPort.Port.kUSB);
  }

  @Provides
  @Named("drive PID")
  public PIDController provideDrivePid() {
    return new PIDController(
        Constants.DriveConstants.DRIVE_P,
        Constants.DriveConstants.DRIVE_I,
        Constants.DriveConstants.DRIVE_D);
  }

  @Provides
  @Singleton
  public DifferentialDriveWheelSpeeds provideWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds();
  }

  /****** Intake Subsystem Dependencies ******/

  @Provides
  @Named("intake pistons")
  public DoubleSolenoid provideIntakePistons() {
    DoubleSolenoid pistons =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            Constants.IntakeConstants.INTAKE_PISTONS_SOLENOID[0],
            Constants.IntakeConstants.INTAKE_PISTONS_SOLENOID[1]);
    pistons.set(DoubleSolenoid.Value.kForward);
    return pistons;
  }

  @Provides
  @Named("intake falcon")
  public WPI_TalonFX provideIntakeFalcon() {
    WPI_TalonFX intakeFalcon = new WPI_TalonFX(Constants.IntakeConstants.INTAKE_FALCON);
    intakeFalcon.setInverted(true);
    return intakeFalcon;
  }

  /****** LED Subsystem Dependencies ******/
  @Provides
  public AddressableLED provideLedStrip() {
    return new AddressableLED(Constants.LEDConstants.ADDRESSABLE_LED);
  }

  @Provides
  public AddressableLEDBuffer provideLedBuffer() {
    return new AddressableLEDBuffer(Constants.LEDConstants.LED_COUNT);
  }

  /****** Neck Subsystem Dependencies ******/

  @Provides
  @Named("front spark")
  public CANSparkMax providesFrontSpark() {
    return new CANSparkMax(
        Constants.NeckConstants.FRONT_INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  @Provides
  @Named("rear spark")
  public CANSparkMax providesRearSpark() {
    return new CANSparkMax(
        Constants.NeckConstants.REAR_INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  @Provides
  @Named("lower beam break")
  public DigitalInput providesLowerBeamBreak() {
    return new DigitalInput(Constants.NeckConstants.NECK_LOWER_BEAM_BREAK);
  }

  @Provides
  @Named("upper beam break")
  public DigitalInput providesUpperBeamBreak() {
    return new DigitalInput(Constants.NeckConstants.NECK_UPPER_BEAM_BREAK);
  }

  /****** Shifter Subsystem Dependencies ******/

  @Provides
  @Named("shifter")
  public DoubleSolenoid providesShifter() {
    return new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        Constants.DriveConstants.SOLENOID_SHIFTER[0],
        Constants.DriveConstants.SOLENOID_SHIFTER[1]);
  }

  /****** Shooter Subsystem Dependencies ******/

  @Provides
  @Named("shooter left")
  public WPI_TalonFX provideShooterLeftFalcon() {
    WPI_TalonFX falcon = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_LEFT_FALCON);
    falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    falcon.setNeutralMode(NeutralMode.Coast);
    falcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    falcon.enableVoltageCompensation(true);

    return falcon;
  }

  @Provides
  @Named("shooter right")
  public WPI_TalonFX provideShooterRightFalcon() {
    WPI_TalonFX falcon = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_RIGHT_FALCON);
    falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    falcon.setNeutralMode(NeutralMode.Coast);
    falcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
    falcon.enableVoltageCompensation(true);

    return falcon;
  }

  @Provides
  @Named("shooter pid")
  public PIDController provideShooterPidController() {
    return new PIDController(
        Constants.ShooterConstants.SHOOTER_P,
        Constants.ShooterConstants.SHOOTER_I,
        Constants.ShooterConstants.SHOOTER_D);
  }

  @Provides
  @Named("feed forward")
  public SimpleMotorFeedforward provideFeedForward() {
    return new SimpleMotorFeedforward(
        Constants.ShooterConstants.SHOOTER_KS,
        Constants.ShooterConstants.SHOOTER_KV,
        Constants.ShooterConstants.SHOOTER_KA);
  }

  /****** Turret Subsystem Dependencies ******/

  @Provides
  @Named("turret spark")
  public CANSparkMax provideTurretSpark() {
    CANSparkMax spark =
        new CANSparkMax(
            Constants.TurretConstants.TURRET_SPARK, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    spark.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward,
        (float) Constants.TurretConstants.TURRET_MAX_RIGHT);
    spark.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.TurretConstants.TURRET_MAX_LEFT);
    return spark;
  }

  @Provides
  @Named("mag switch")
  public DigitalInput provideMagSwitch() {
    return new DigitalInput(Constants.TurretConstants.MAG_SWITCH_PORT);
  }

  @Provides
  public AutoHelper providesAutoBuilder() {
    return new AutoHelper();
  }
}
