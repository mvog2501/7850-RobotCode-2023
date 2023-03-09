package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;

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

    //testing if the motors work
    public void testUp() {
        armMotor1.set(0.2);
        armMotor2.follow(armMotor1);


    }

    public void testDown() {
        armMotor1.set(-0.2);
        armMotor2.follow(armMotor1);
    }

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
        PIDController armHorizPID = new PIDController(0, 0, 0);
        PIDController armVertPID = new PIDController(0, 0, 0);
        

    }

    //extends/retracts arm
    public void TestExtension() {
        armMotor3.set(0.5);
    }

    public void TestRetraction() {
        armMotor3.set(-0.5);
    }

    public void stopVertMotors() {

        armMotor1.set(0);
        armMotor2.set(0);

    }

    public void stopHorizMotors() {

        armMotor3.set(0);

    }

   
}
