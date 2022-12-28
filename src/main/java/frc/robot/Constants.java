// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

import static java.lang.Math.PI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int DRIVE_TALON_BL_ID = 3;
    public static int DRIVE_TALON_BR_ID = 4;
    public static int DRIVE_TALON_FL_ID = 2;
    public static int DRIVE_TALON_FR_ID = 1;

    public static int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
    public static double kGearRatio = 10.71;
    public static double kWheelRadiusInches = 4;
    public static int k100msPerSecond = 10;

    public static final double ksVolts = 0.63575;
    public static final double kvVoltSecondsPerMeter = 2.713*0.335;
    public static final double kaVoltSecondsSquaredPerMeter = 0.18978;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = .702;

    public static final double kTrackwidthMeters = 0.61857;
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (Units.inchesToMeters(kWheelRadiusInches)*2*PI) / (double) kCountsPerRev / kGearRatio;

}
