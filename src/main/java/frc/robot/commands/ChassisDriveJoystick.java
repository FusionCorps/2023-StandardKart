package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class ChassisDriveJoystick extends CommandBase {

    Chassis m_chassis;
    XboxController m_controller;

    public ChassisDriveJoystick(Chassis chassis, XboxController controller) {
        m_chassis = chassis;
        m_controller = controller;

        addRequirements(m_chassis);
    }

    @Override
    public void execute() {
        m_chassis.curvatureDrive(-m_controller.getLeftY(), 0.6*m_controller.getRightX());
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.curvatureDrive(0, 0);
    }
    
}
