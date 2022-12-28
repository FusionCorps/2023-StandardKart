package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveJoystick extends CommandBase {

    Chassis m_chassis;
    XboxController m_controller;

    SlewRateLimiter fwd_limiter = new SlewRateLimiter(6.0);
    SlewRateLimiter rot_limiter = new SlewRateLimiter(6.0);

    public ChassisDriveJoystick(Chassis chassis, XboxController controller) {
        m_chassis = chassis;
        m_controller = controller;

        addRequirements(m_chassis);
    }

    @Override
    public void execute() {
        m_chassis.curvatureDrive(-fwd_limiter.calculate(m_controller.getRawAxis(1)), rot_limiter.calculate(0.5*m_controller.getRawAxis(0)));
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.curvatureDrive(0, 0);
    }
    
}
