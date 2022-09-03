package frc.robot.di;

import dagger.Component;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.TurretLockCommand;

import javax.inject.Singleton;

@Singleton
@Component(modules = {SubsystemsModule.class, CommandsModule.class, RobotModule.class})
public interface RobotComponent {
  @Component.Builder
  interface Builder {
    public RobotComponent build();
  }

  void inject(Robot robot);
  RobotContainer container();
  TurretLockCommand turretLockCommand();
}
