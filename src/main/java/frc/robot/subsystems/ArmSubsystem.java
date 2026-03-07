package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ArmConstants.ArmUpwardsHighGravityPID;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax ArmMotor;

    private final SparkClosedLoopController armPidController;
    private final SparkRelativeEncoder armEncoder;

    private final double gravityFF = 0.07;
 
    private Double setPosition;

    private double percentOut = 0;

    private double pseudoBottomLimit = -10;
    private double pseudoTopLimit = 5;
    
   public ArmSubsystem(){
    
SparkMaxConfig config = new SparkMaxConfig();
config.closedLoop.pid(
    Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kP,
Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kI,
Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kD,
ClosedLoopSlot.kSlot0);
config.smartCurrentLimit(20);
config.idleMode(IdleMode.kCoast);


//motor
    ArmMotor = new SparkMax(Constants.ARM_MOTOR,SparkLowLevel.MotorType.kBrushless);

    ArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//PID controloer 
    armPidController = ArmMotor.getClosedLoopController();
// encodoer 
    armEncoder = (SparkRelativeEncoder) ArmMotor.getEncoder();
//geting position of the encoder 
    armEncoder.setPosition(0.0);

    setPosition = armEncoder.getPosition();
    // figuer out how to make this go to the corect posithion me 

        
        //armPidController.setSetpoint(setPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    // public void Intakedown(){
    //     armPidController.setSetpoint(Constants.ArmConstants.ARM_AT_NEUTRAL_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //     ArmMotor.set(-.5);
    // }
    public Command armSetPoints(double setState){
        return new InstantCommand(()-> 
        armPidController.setSetpoint(setState,ControlType.kVoltage,ClosedLoopSlot.kSlot0));
    }

        public void setArmMotorSpeed(double speed){
        ArmMotor.set(speed);
    }
    
    public Command ArmIntake(){
        // return new InstantCommand(()-> Intakedown());
        return new StartEndCommand(
        () -> ArmMotor.set(-0.2),  // start
        () -> ArmMotor.set(0),    // stop
        this
    ).withTimeout(2.0);
    }
   
    public Command armToNeutralLevel(){
    // return new InstantCommand(() -> armPidController.setSetpoint(Constants.ArmConstants.ARM_AT_NEUTRAL_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0));
        if (setPosition == 0){
            return new InstantCommand(()-> ArmMotor.stopMotor());
        }
    return new InstantCommand(
        () -> ArmMotor.set(.3));
    }

} 
