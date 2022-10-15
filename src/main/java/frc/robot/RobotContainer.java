// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.LimelightCar;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LimeLightSubsystem m_limelight = new LimeLightSubsystem();
  //private final LimeLightSubsystem m_limelight = new LimeLightSubsystem();

  Joystick m_driverController = new Joystick(OIConstants.DriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.arcadeDrive(
          -0.4*m_driverController.getRawAxis(1), 0.6*m_driverController.getRawAxis(4)
        ),m_robotDrive
      )
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, 6)
      .whenPressed(() -> m_robotDrive.setMaxOutput(OIConstants.lofi_output))
      .whenReleased(() -> m_robotDrive.setMaxOutput(OIConstants.hifi_output));
    new JoystickButton(m_driverController, 1).whenHeld(new LimelightAim(m_robotDrive, m_limelight));
    new JoystickButton(m_driverController, 2).whenPressed(() -> CommandScheduler.getInstance().cancelAll());
    new JoystickButton(m_driverController, 3).whenPressed(() -> new LimelightCar(m_robotDrive, m_limelight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new RunCommand(()->CommandScheduler.getInstance().schedule(new LimelightAim(m_robotDrive, m_limelight)),m_robotDrive,m_limelight);
  }

  public void testDrive(){
    //m_robotDrive.testMotor();
  }

  public void stopTest(){
    //m_robotDrive.stopMotor();
  }
}
