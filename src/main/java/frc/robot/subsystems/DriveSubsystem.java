// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX FL = new WPI_TalonSRX(DriveConstants.FrontLeftMotorPort);
  private final WPI_TalonSRX RL = new WPI_TalonSRX(DriveConstants.RearLeftMotorPort);
  private final WPI_TalonSRX FR = new WPI_TalonSRX(DriveConstants.FrontRightMotorPort);
  private final WPI_TalonSRX RR = new WPI_TalonSRX(DriveConstants.RearRightMotorPort);

  private final MotorControllerGroup m_LeftMotors = new MotorControllerGroup(FL, RL);
  private final MotorControllerGroup m_RightMotors = new MotorControllerGroup(FR, RR);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_LeftMotors, m_RightMotors);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    
  }

  public void arcadeDrive(double fwd, double rot) {
    arcadeDrive(fwd, rot, false);
  }

  public void arcadeDrive(double speed, double rotation, boolean useSquares){
    m_drive.arcadeDrive(speed, rotation, useSquares);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  } 
}
