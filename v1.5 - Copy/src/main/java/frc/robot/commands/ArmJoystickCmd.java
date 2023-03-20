package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.OperatorConstants.ArmConstants;
import frc.robot.subsystems.RobotArmSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ArmJoystickCmd extends CommandBase {
    private final RobotArmSubsystem robotArmSubsystem;
    private final Supplier<Double> speedFunction;
    private final SlewRateLimiter limiter;

    /**
     * @param robotArmSubsystem
     * @param speedFunction
     */
    public ArmJoystickCmd(RobotArmSubsystem robotArmSubsystem, Supplier<Double> speedFunction) {
        this.robotArmSubsystem = robotArmSubsystem;
        this.speedFunction = speedFunction;
        this.limiter = new SlewRateLimiter(ArmConstants.maxArmAccel);

        addRequirements(robotArmSubsystem);
    }

   

    // private void addRequirements(RobotArmSubsystem robotArmSubsystem) {
    // }


    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
     // 1. Get real-time joystick inputs
    double speed = speedFunction.get();

    // 2. Apply deadband (if the joystick movements are too small the inputs are ignored)
    speed = Math.abs(speed) > ArmConstants.kArmDeadband ? speed : 0.0; 
    

    // 3. Make the motion smoother with rate limiter
    speed = limiter.calculate(speed) * ArmConstants.armMaxSpeed;
    
    // 4. Apply speed to motors
    robotArmSubsystem.setVertMotors(speed - 0.01);
 
    }

    @Override
    public void end(boolean interrupted) {
        robotArmSubsystem.stopVertMotors();
       
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}