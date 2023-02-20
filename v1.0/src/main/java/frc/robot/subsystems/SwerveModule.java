// Class to create states for the four Swerve Modules
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.OperatorConstants.ModuleConstants;
//import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants.DriveConstants;


public class SwerveModule {
// Create the variables for the module
    
    // Motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    // Encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    // PID Controller
    private final PIDController turningPidController;

    // Absolute Encoder
    private final CANCoder cancoder;
    // private final AnalogInput absoluteEncoder;
    // private final boolean absoluteEncoderReversed; // Is the Absolute Encoder Reversed?
    // private final double absoluteEncoderOffsetRad; // How much the Absolute Encoder is off by in radians

    /**
     * @param driveMotorId
     * @param turningMotorId
     * @param driveMotorReversed
     * @param turningMotorReversed
     * @param absoluteEncoderId
     * @param absoluteEncoderOffset
     * @param absoluteEncoderReversed
     */

    // Constructor used to gather the information for each module
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset) {
        // Initializing the variables to the paramaters of each 

        // this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        // this.absoluteEncoderReversed = absoluteEncoderReversed;
        // absoluteEncoder = new AnalogInput(absoluteEncoderId);
        cancoder = new CANCoder(absoluteEncoderId);

        //cancoder.configMagnetOffset(Math.toDegrees(absoluteEncoderOffset));
        cancoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        cancoder.configMagnetOffset(absoluteEncoderOffset);
        cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        //cancoder.setPosition(Math.toDegrees(absoluteEncoderOffset)); 


        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        // Set the encoder conversion constants
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        
        // Created PID Controller, only uses porportional term because it does the job pretty well
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

        // Makes the PID Controller Continous 
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);


            }

    // Gets position of the driving motor
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    // Gets position of the turning motor
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    // Gets the driving motor's velocity
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    // Gets the turning motor's velocity
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getCancoderPosition()
    {
        return cancoder.getAbsolutePosition();
    }

    // Calculates the angle of the Absolute Encoder in radians
    // public double getAbsoluteEncoderRad() {
    //     double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    //     angle *= 2.0 * Math.PI;
    //     angle -= absoluteEncoderOffsetRad;
    //     return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    // }

    // Reset Encoders
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(Math.toRadians(cancoder.getAbsolutePosition()));
    }

    // Returns the state of the motors (velocity, turning position)
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    // Returns the SwerveModulePosition to use fo the odometers (distance, turning position)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    // Sets each Swerve Module to their desired states
    public void setDesiredState(SwerveModuleState state) {

        // Prevents the modules from resetting when the speed is too low
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle); // Optimizes the angle to move less than 90 degrees in either direction
        
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // Sets driving motor speeds
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians())); // Sets turning motor speed

        //SmartDashboard.putString("Swerve[" + cancoder.getAbsolutePosition() + "] state", state.toString());
    }

    // Stops Motors
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);


        
    }
}
            
