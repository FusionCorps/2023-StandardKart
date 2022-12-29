// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChassisDriveJoystick;
import frc.robot.commands.ChassisVoltsTime;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PathFollower;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  XboxController m_controller = new XboxController(0);

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Chassis m_chassis = new Chassis();

  String trajectoryJSON = "paths/Leg1.wpilib.json";

  String leg2JSON = "paths/Leg2.wpilib.json";
  String leg3JSON = "paths/Leg3.wpilib.json";
  String leg4JSON = "paths/Leg4.wpilib.json";
  String leg5JSON = "paths/Leg5.wpilib.json";

  Trajectory trajectory = new Trajectory();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_chassis.setDefaultCommand(new ChassisDriveJoystick(m_chassis, m_controller));


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
//    new JoystickButton(m_controller, XboxController.Button.kRightBumper.value)
//            .whileHeld(new ChassisVoltsTime(m_chassis, 3.0, 1.0, 1.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(leg2JSON);
      Trajectory trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(leg3JSON);
      Trajectory trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(leg4JSON);
      Trajectory trajectory4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(leg5JSON);
      Trajectory trajectory5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

      trajectory = trajectory1.concatenate(trajectory2.concatenate(trajectory3.concatenate(trajectory4.concatenate(trajectory5))));

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return new PathFollower(m_chassis, trajectory);
  }
}
