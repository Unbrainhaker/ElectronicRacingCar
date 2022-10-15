package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimeLightSubsystem extends SubsystemBase {

    /** Creates a new Subsystem. */
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double x;
    double y;
    double area;
    double target;

    // calculate angles
    double angleToGoalDegrees = LimelightConstants.limelightMounAngleDegrees + y;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    // calculate distance
    double distanceFromLimeligtToGoal = ((LimelightConstants.goalHeightMeters - LimelightConstants.limelightLensHeightMeters)/Math.tan(angleToGoalRadians));

    // proportional control constant for distance
    public void periodic() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        target = tv.getDouble(0.0);


        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        distanceFromLimeligtToGoal = ((LimelightConstants.goalHeightMeters - LimelightConstants.limelightLensHeightMeters)/Math.tan((angleToGoalDegrees+y)*Math.PI/180));
        
        // Should be an command but not in subsystem
        // if (true) {
        //     double desired_distance = 60.0;
        //     double distance_error = desired_distance - current_distance;
        //     double driving_adjust = KpDistance * distance_error;

        //     left_command += distance_adjust;
        //     right_command += distance_adjust;
        // }
    }

    public double getDis() {
        return distanceFromLimeligtToGoal;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getTarget(){
        return target;
    }
}