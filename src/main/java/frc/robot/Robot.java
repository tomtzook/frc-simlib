package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.DriveSystem2;
import frc.robot.subsystems.DriveSystem3;

public class Robot extends TimedRobot {

    private DriveSystem3 driveSystem;

    @Override
    public void robotInit() {
        driveSystem = new DriveSystem3();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {
        driveSystem.drive(0.1, 0.1);
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {
        driveSystem.stop();
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }
}
