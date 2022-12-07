package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisVoltsTime extends CommandBase {

    Chassis m_chassis;

    double left_volts;
    double right_volts;

    Timer m_timer = new Timer();
    double timeout;

    public ChassisVoltsTime(Chassis chassis, double l_volts, double r_volts, double time) {
        m_chassis = chassis;

        left_volts = l_volts;
        right_volts = r_volts;

        timeout = time;

        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_chassis.setDriveVolts(left_volts, right_volts);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(timeout);
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.setDriveVolts(0, 0);
    }

}
