// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants.DriveConstants;
//import frc.robot.Constants.OperatorConstants.JoystickConstants;
//import frc.robot.RobotContainer;


public class SwerveSubsystem extends SubsystemBase {
 
  // Creating the four Swerve Modules
  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveMotorPort,
    DriveConstants.kFrontLeftTurningMotorPort,
    DriveConstants.kFrontLeftDriveEncoderReversed,
    DriveConstants.kFrontLeftTurningEncoderReversed,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
    DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDeg
    );

  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveMotorPort,
    DriveConstants.kFrontRightTurningMotorPort,
    DriveConstants.kFrontRightDriveEncoderReversed,
    DriveConstants.kFrontRightTurningEncoderReversed,
    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
    DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDeg);

  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveMotorPort,
    DriveConstants.kBackLeftTurningMotorPort,
    DriveConstants.kBackLeftDriveEncoderReversed,
    DriveConstants.kBackLeftTurningEncoderReversed,
    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDeg);

  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.kBackRightDriveMotorPort,
    DriveConstants.kBackRightTurningMotorPort,
    DriveConstants.kBackRightDriveEncoderReversed,
    DriveConstants.kBackRightTurningEncoderReversed,
    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
    DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDeg);

    // Creating the gyroscope
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Odometry
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0),
     new SwerveModulePosition[] {
       frontLeft.getPosition(),
       frontRight.getPosition(),
       backLeft.getPosition(),
       backRight.getPosition()});

    public SwerveSubsystem() {
      frontLeft.resetEncoders();
      frontRight.resetEncoders();
      backLeft.resetEncoders();
      backRight.resetEncoders();
        new Thread(() -> { // Waits for 1s after boot to zero the gyroscope
          try {
              Thread.sleep(1000);
              zeroHeading();
          } catch (Exception e) {
          }
      }).start();
    }

    // Resets the robot's heading to 0 degrees
    public void zeroHeading() {
        gyro.reset();
    }
    
    // Gets the robot's heading (from -180 to 180 degrees)
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    // Converts the heading to Rotation2d
    public Rotation2d getRotation2d() {
       return Rotation2d.fromDegrees(getHeading());
    }
 
    // Gets the position of the robot on the field
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // Resets the odometry
    public void resetOdometry(Pose2d pose) {
       odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()}, pose);
    }


  @Override
  public void periodic() {
  // Debug info
  // Odometer Update
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
       frontLeft.getPosition(),
       frontRight.getPosition(),
       backLeft.getPosition(),
       backRight.getPosition()});
        
      // Heading and Location Debug Info
      SmartDashboard.putNumber("Robot Heading", getHeading());
      SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

      // Absolute Encoder Debug Info
      SmartDashboard.putNumber("AERFL", frontLeft.getCancoderPosition());
      SmartDashboard.putNumber("AERFR", frontRight.getCancoderPosition());
      SmartDashboard.putNumber("AERBL", backLeft.getCancoderPosition());
      SmartDashboard.putNumber("AERBR", backRight.getCancoderPosition());

      // Turning Motor Position Debug Info
      SmartDashboard.putNumber("FLTER", frontLeft.getTurningPosition());
      SmartDashboard.putNumber("FRTER", frontRight.getTurningPosition());
      SmartDashboard.putNumber("BLTER", backLeft.getTurningPosition());
      SmartDashboard.putNumber("BRTER", backRight.getTurningPosition());

      // Turning Motor Velocity Debug Info
      SmartDashboard.putNumber("FLTVR", frontLeft.getTurningVelocity());
      SmartDashboard.putNumber("FRTVR", frontRight.getTurningVelocity());
      SmartDashboard.putNumber("BLTVR", backLeft.getTurningVelocity());
      SmartDashboard.putNumber("BRTVR", backRight.getTurningVelocity());

      // Drive Motor Encoder Debug Info
      SmartDashboard.putNumber("FLDER", frontLeft.getDrivePosition());
      SmartDashboard.putNumber("FRDER", frontRight.getDrivePosition());
      SmartDashboard.putNumber("BLDER", backLeft.getDrivePosition());
      SmartDashboard.putNumber("BRDER", backRight.getDrivePosition());

      // Drive Motor Velocity Debug Info
      SmartDashboard.putNumber("FLDVR", frontLeft.getDriveVelocity());
      SmartDashboard.putNumber("FRDVR", frontRight.getDriveVelocity());
      SmartDashboard.putNumber("BLDVR", backLeft.getDriveVelocity());
      SmartDashboard.putNumber("BRDVR", backRight.getDriveVelocity());

    

  }

  // Stops all modules
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

// Sets each module to its desired state + normalizing wheel speeds
public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); 
  frontLeft.setDesiredState(desiredStates[0]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[3]);
}
  
}
