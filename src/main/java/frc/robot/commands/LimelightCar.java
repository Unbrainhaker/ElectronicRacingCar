package frc.robot.commands;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightCar extends CommandBase{
    
    private static final double GGdistance = 3.0;

    public double toWallDistance;
    
    private DriveSubsystem m_drive;
    private LimeLightSubsystem m_limelight;

    Joystick m_driverController = new Joystick(OIConstants.DriverControllerPort);

    public LimelightCar(DriveSubsystem driveSubsystem, LimeLightSubsystem limelight) {
        m_drive = driveSubsystem;
        m_limelight = limelight;
        addRequirements(driveSubsystem);
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        toWallDistance = m_limelight.getDis();
    }

    @Override
    public void execute() {
        toWallDistance = m_limelight.getDis();
        if(toWallDistance >= GGdistance) {
            m_drive.arcadeDrive(-0.4*m_driverController.getRawAxis(1), 0.6*m_driverController.getRawAxis(4));
        } else {
            m_drive.arcadeDrive(0, 0);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_drive.arcadeDrive(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        if(toWallDistance < GGdistance) {
           return true; 
        } else {
            toWallDistance = m_limelight.getDis();
            return false;
        }
    }
}
