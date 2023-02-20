// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static class ModuleConstants {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1 / 8.14; 
      public static final double kTurningMotorGearRatio = 1 / 21.4285714286; 
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
      public static final double kPTurning = 0.5; // Porportional term of the PID Controller for the turning motor

    }

    public static class DriveConstants {
      public static final double kTrackWidth = Units.inchesToMeters(23); // Distance between right and left wheels 
      public static final double kWheelBase = Units.inchesToMeters(23); // Distance between the front and back wheels
      
      // Locations of each Swerve Module on the Robot
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // frontLeft
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // frontRight
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // backLeft
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // backRight

      public static final int kFrontLeftDriveMotorPort = 3; 
      public static final int kBackLeftDriveMotorPort = 12; 
      public static final int kFrontRightDriveMotorPort = 6; 
      public static final int kBackRightDriveMotorPort = 9; 

      public static final int kFrontLeftTurningMotorPort = 1; 
      public static final int kBackLeftTurningMotorPort = 10; 
      public static final int kFrontRightTurningMotorPort = 4; 
      public static final int kBackRightTurningMotorPort = 7; 

      public static final boolean kFrontLeftTurningEncoderReversed = false; // Determine our values (neg = reversed)
      public static final boolean kBackLeftTurningEncoderReversed = false; // Determine our values (neg = reversed)
      public static final boolean kFrontRightTurningEncoderReversed = false; // Determine our values (neg = reversed)
      public static final boolean kBackRightTurningEncoderReversed = false; // Determine our values (neg = reversed)

      public static final boolean kFrontLeftDriveEncoderReversed = false; // Determine our values (neg = reversed)
      public static final boolean kBackLeftDriveEncoderReversed = false; // Determine our values (neg = reversed)
      public static final boolean kFrontRightDriveEncoderReversed = false; // Determine our values (neg = reversed)
      public static final boolean kBackRightDriveEncoderReversed = false; // Determine our values (neg = reversed)

      public static final int kFrontLeftDriveAbsoluteEncoderPort = 2; 
      public static final int kBackLeftDriveAbsoluteEncoderPort = 11; 
      public static final int kFrontRightDriveAbsoluteEncoderPort = 5; 
      public static final int kBackRightDriveAbsoluteEncoderPort = 8; 

      // public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false; // Determine our values (neg = reversed)
      // public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false; // Determine our values (neg = reversed)
      // public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false; // Determine our values (neg = reversed)
      // public static final boolean kBackRightDriveAbsoluteEncoderReversed = false; // Determine our values (neg = reversed)

      public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = -119.71; // Determine our values
      public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = -194.50; // Determine our values
      public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = -264.11; // Determine our values
      public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = -175.61; // Determine our values

      public static final double kPhysicalMaxSpeedMetersPerSecond = 5; // Determine our values/Guess
      public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // Determine our values/Guess

      public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
      public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
              
      public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3; // Determine our values/Guess
      public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3; // Determine our values/Guess
    }

    public static class JoystickConstants {
      public static final int rightStickPort = 0;
      public static final int leftStickPort = 1;

      public static final int kDriverXAxis = 0;
      public static final int kDriverYAxis = 1;
      public static final int kDriverRotAxis = 2;
      public static final int kDriverFieldOrientedButtonIdx = 5; // Find whatever button works the best

      public static final double kDeadband = 0.12;
    }

    // public static class RightStickButtonPort {
    //   public static final int trigger = 1;
    //   public static final int topLeft = 2;
    // }

    // public static class LeftStickButtonPort {
    //   public static final int topMiddle = 2;
    // }

    // public static class RightJoystickAxes {
    //   public static final int rightXAxis = 0;
    //   public static final int rightYAxis = 1;
    //   public static final int rightZAxis = 2;
    // }

    // public static class LeftJoystickAxes {
    //   public static final int leftXAxis = 0;
    //   public static final int leftYAxis = 1;
    //   public static final int leftZAxis = 2;
    // }

  }
}
