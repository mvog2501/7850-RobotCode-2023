package frc.robot.subsystems;
import java.beans.VetoableChangeSupport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants.ArmConstants;

// import frc.robot.commands.ArmUpCmd;


public class RobotArmSubsystem {
    //Motors
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;
    private final CANSparkMax armMotor3;

    //Motor Encoders
    private final RelativeEncoder armMotor1Encoder;
    private final RelativeEncoder armMotor2Encoder;
    private final RelativeEncoder armMotor3Encoder;

    private double horizSpeed = 0.75;
    private double vertSpeed = 0.2;

    private double setpoint;

    //creating the command
    // public void setDefaultCommand(ArmUpCmd armOpenCmd) {
    // }

    public RobotArmSubsystem(){

        //Motor id's
        armMotor1 = new CANSparkMax(44, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(45, MotorType.kBrushless);
        armMotor3 = new CANSparkMax(46, MotorType.kBrushless);

        //Encoders
        armMotor1Encoder = armMotor1.getEncoder();
        armMotor2Encoder = armMotor2.getEncoder();
        armMotor3Encoder = armMotor3.getEncoder();

        //PID controllers
        PIDController armHorizPID = new PIDController(ArmConstants.hP, ArmConstants.hI, ArmConstants.hD);
        PIDController armVertPID = new PIDController(ArmConstants.vP, ArmConstants.vI, ArmConstants.vD);


        // Debug Info
        SmartDashboard.putNumber("ArmMotor1 Pos", armMotor1Encoder.getPosition());
        SmartDashboard.putNumber("ArmMotor2 Pos", armMotor2Encoder.getPosition());
        SmartDashboard.putNumber("ArmMotor3 Pos", armMotor3Encoder.getPosition());
        // SmartDashboard.putNumber("ArmMotor1 Velo", armMotor2Encoder.getVelocity());
        // SmartDashboard.putNumber("ArmMotor2 Velo", armMotor3Encoder.getVelocity());
        // SmartDashboard.putNumber("ArmMotor3 Velo", armMotor3Encoder.getVelocity());

    }

    //testing if the motors work
    public void testUp() {
       armMotor1.set(vertSpeed);
       armMotor2.set(-vertSpeed);
    }

    public void testDown() {
        armMotor1.set(-vertSpeed);
        armMotor2.set(vertSpeed);
    }

    //extends/retracts arm
    public void testExtension() {
        armMotor3.set(horizSpeed);
    }

    public void testRetraction() {
        armMotor3.set(-horizSpeed);
    }

    public void stopVertMotors() {

        armMotor1.set(0);
        armMotor2.set(0);

    }

    public void stopHorizMotors() {

        armMotor3.set(0);

    }

    public void setVertMotors(double speed){
        armMotor1.set(speed);
        armMotor2.set(-speed);
    }

    public void setHorizMotors(double speed){
        armMotor3.set(speed);
    }

    

   
}
