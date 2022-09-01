package frc.robot.di;

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import javax.inject.Singleton;

@Module
public class RobotModule {
  @Singleton
  @Provides
  public RobotContainer provideRobotContainer() {
    return new RobotContainer();
  }

  @Provides
  public PowerDistribution provideRevPDH() {
    return new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
  }

  @Provides
  public Joystick provideJoystick() {
    return new Joystick(Constants.OIConstants.DRIVER_STATION_JOY);
  }
}
