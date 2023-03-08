package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



// import frc.robot.commands.ArmUpCmd;


public class RobotArmSubsystem {
    //Motors
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;
    private final CANSparkMax armMotor3;

    //testing if the motors work
    public void testUp() {
        armMotor1.set(0.1);
        armMotor2.follow(armMotor1);


    }

    public void testDown() {
        armMotor1.set(-0.1);
        armMotor2.follow(armMotor1);
    }

    //creating the command
    // public void setDefaultCommand(ArmUpCmd armOpenCmd) {
    // }

    //motor id's
    public RobotArmSubsystem(){

        armMotor1 = new CANSparkMax(44, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(45, MotorType.kBrushless);
        armMotor3 = new CANSparkMax(46, MotorType.kBrushless);
        

    }

    //extends/retracts arm
    public void TestExtension() {
        armMotor3.set(0.1);
    }

    public void TestRetraction() {
        armMotor3.set(-0.1);
    }

    
}
