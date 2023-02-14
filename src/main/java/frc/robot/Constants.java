// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
    public static final double kMovementMultiplier = 0.5;

    public static final int kLeftMotor1Port = 5;
    public static final int kLeftMotor2Port = 4;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final double kTrackwidthMeters = 0.62898; //Need to measure
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524; // 6in wheel diameter
    public static final double kGearRatio = (double) 111/11; //Change I think
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (kEncoderCPR * kGearRatio);

    public static final double ksVolts = 0.25355;
    public static final double kvVoltSecondsPerMeter = 2.0519;
    public static final double kaVoltSecondsSquaredPerMeter = 0.53639;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 3.7728;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kTurnP = 0.8;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.0;
    public static final double kTurnToleranceDeg = 1.0;
    public static final double kTurnRateToleranceDegPerS = 1.0;

    // Placeholder Mass
    public static final double kMass = 100.0;
  }
  
  public static final class PneumaticConstants {
        public static final int compressorPort = 1;

        //ids for compressors and solenoids
        public static final int extendSolenoidPortLeft = 0;
        public static final int retractSolenoidPortLeft = 1;
        public static final int extendSolenoidPortRight = 2;
        public static final int retractSolenoidPortRight = 3;
    
        //movement constants
        public static final double extendSpeed = 0.1;
        public static final double retractSpeed = 0.1;
  }
}
