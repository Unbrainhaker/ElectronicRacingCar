// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int FrontLeftMotorPort = 1;
        public static final int RearLeftMotorPort = 3;
        public static final int FrontRightMotorPort = 4;
        public static final int RearRightMotorPort = 2;
    }

    public static final class OIConstants {
        public static final int DriverControllerPort = 0;
        public static final double lofi_output = 0.45;
        public static final double hifi_output = 0.9;
    }
    
    public static final class LimelightConstants {
        // how many degrees back is your limelight rotateed from perfectly vertical
    // check our own limelight degree
    public static final double limelightMounAngleDegrees = 52.0;

    // distance from the center of the limelight lens to the floor
    public static final double limelightLensHeightMeters = 0.24;

    // distance from the target to the floor
    public static final double goalHeightMeters = 1;

    // define constants
    public static final double KpDistance = -0.1f;
    public static final double KpAim = 0;
    public static final double min_command = 0;
    public static final double current_distance = 0; // distanceFromLimeligtToGoalInches
  }
}
