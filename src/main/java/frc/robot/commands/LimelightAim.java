// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class LimelightAim extends CommandBase {

  private static final double kPangle = 1.65;
  private static final double kIangle = 0.005;
  private static final double kDangle = 0.0001;
  private static final double timeStamp = 0.02;

  private static final double kPdistance = 0.4;
  private static final double kIdistance = 0.0005;
  private static final double kDdistance = 0.00001;

  private double target;

  private double distanceError;
  private double distanceErrorSum;
  private double lastDistanceError;

  private double angleError;
  private double angleErrorSum;
  private double lastAngleError;
  
  private DriveSubsystem m_drive;
  private LimeLightSubsystem m_limelight;


  public LimelightAim(DriveSubsystem driveSubsystem, LimeLightSubsystem limelight) {
    m_drive = driveSubsystem;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleError = m_limelight.getX()*Math.PI/180;
    distanceError = m_limelight.getDis();
    System.out.println("Executing Limelight Aim......");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = m_limelight.getTarget();

    angleError = m_limelight.getX()*Math.PI/180;
    distanceError = m_limelight.getDis();
    if(Math.abs(angleErrorSum) < 100) {
      angleErrorSum += angleError;
    }
    if(Math.abs(distanceErrorSum) < 1000) {
      distanceErrorSum += distanceError;
    }

    double distanceDerivative = (distanceError - lastDistanceError) / timeStamp;
    double angleDerivative = (angleError - lastAngleError) / timeStamp;

    double angleOutput = kPangle * angleError + kIangle * angleErrorSum + kDangle * angleDerivative;
    double distanceOutput = kPdistance * distanceError + kIdistance * distanceErrorSum + kDdistance * distanceDerivative;

    if(target > 0) {
      m_drive.arcadeDrive(distanceOutput, angleOutput);
    } else {
      m_drive.arcadeDrive(0, 0);
    }

    lastAngleError = angleError;
    lastDistanceError = distanceError;
    SmartDashboard.putNumber("distanceOutput", distanceOutput);
    SmartDashboard.putNumber("distanceError", distanceError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_drive.arcadeDrive(0, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(angleError) < 0.1 && Math.abs(distanceError) < 0.3) {
      return true;
    }
    angleErrorSum = 0;
    distanceErrorSum = 0;
    return false;
  }
}
