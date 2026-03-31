package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ArmConstants.ArmUpwardsHighGravityPID;

public class ArmSubsystem extends SubsystemBase {

    private double setPosition;

    private final SparkMax ArmMotor;

    private final SparkClosedLoopController armPidController;
    private final SparkRelativeEncoder armEncoder;

    private final double gravityFF = 0.07;

    private double percentOut = 0;

    private double pseudoBottomLimit = -10;
    private double pseudoTopLimit = 5;
    private final CANcoder armCanEncoder;
    
   public ArmSubsystem(){
    
SparkMaxConfig config = new SparkMaxConfig();
config.closedLoop.pid(
    Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kP,
Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kI,
Constants.ArmConstants.ARM_UPWARDS_HIGH_GRAVITY_PID.kD,
ClosedLoopSlot.kSlot0);
config.smartCurrentLimit(20);
config.idleMode(IdleMode.kCoast);

//Lucy edits ._.
//possible issue?
// config.softLimit.forwardSoftLimit(Constants.ArmConstants.ARM_LOWER_LIMIT);
// config.softLimit.reverseSoftLimit(Constants.ArmConstants.ARM_UPPER_LIMIT);
// config.softLimit.forwardSoftLimitEnabled(true);
// config.softLimit.reverseSoftLimitEnabled(true);

config.closedLoop.feedForward
    .kS(0)
    .kV(0)
    .kA(0);

//motor
    ArmMotor = new SparkMax(Constants.ARM_MOTOR, SparkLowLevel.MotorType.kBrushless);

    ArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//PID controloer 
    armPidController = ArmMotor.getClosedLoopController();
// encodoer 
    armEncoder = (SparkRelativeEncoder) ArmMotor.getEncoder();

    armCanEncoder = new CANcoder(Constants.ARM_CAN_ENCODER);
    double positionEncoder = armCanEncoder.getAbsolutePosition().getValueAsDouble();
//geting position of the encoder 
    armEncoder.setPosition(0.0);

    setPosition = armEncoder.getPosition();
    // figuer out how to make this go to the corect posithion me 

        
        //armPidController.setSetpoint(setPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }// end of constructer
    // public void Intakedown(){
    //     armPidController.setSetpoint(Constants.ArmConstants.ARM_AT_NEUTRAL_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //     ArmMotor.set(-.5);
    // }
public double armDegreesToMotorRotations(double degrees) {
        //System.out.println("Setting Arm Position: " + (-degrees*motorRotationsPerArmDegree));
        return degrees * ARM_MAX_ROTATIONS;
    }

    public void resetMotorEncoderToAbsolute() {
        double newPosition = getArmDegrees() * ARM_MAX_ROTATIONS;

        armEncoder.setPosition(newPosition);
    }

    public void rotateToDegrees(double degrees) {
        setPosition = armDegreesToMotorRotations(degrees);
    }

    public void rotateToMotorRotations(double rotations) {
        setPosition = rotations;
    }

    public double getArmDegrees() {
        return armCanEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    public void rotatePercentOut(double percentOut) {
        this.percentOut = percentOut;
    }
    public Command armSetPoints(double setState){
        return new InstantCommand(()-> 
        armPidController.setSetpoint(setState,ControlType.kPosition,ClosedLoopSlot.kSlot0));
    }

        public Command setArmMotorSpeed(double speed){
        return new InstantCommand(() -> ArmMotor.set(speed));
    }
    
    public Command ArmIntake(){
        // return new InstantCommand(()-> Intakedown());
        return new StartEndCommand(
        () -> ArmMotor.set(-0.5),  // start
        () -> ArmMotor.set(0),// stop
        this
    ).withTimeout(0.83);
    }
   /*public Command ArmWiggle(){
         return new InstantCommand(() ->ArmMotor.set(-0.1));
   }*/
    public Command armToNeutralLevel(){
    // return new InstantCommand(() -> armPidController.setSetpoint(Constants.ArmConstants.ARM_AT_NEUTRAL_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0));
        if (setPosition == 0){
            return new InstantCommand(()-> ArmMotor.stopMotor());
        }
    return new InstantCommand(
        () -> ArmMotor.set(.3));
    }
    double ARM_MAX_ROTATIONS = Constants.ArmConstants.ARM_MAX_ROTATIONS;
    // public void spinByJostick( double amount){
    //     double spinAmount = MathUtil.applyDeadband(amount * ARM_MAX_ROTATIONS,.1);
    //     double sinscaler =  Math.sin(Math.toRadians(amount));
    //     double feedForward = gravityFF * sinscaler;
        
    // }
    public void periodic(){
        // TODO: add && !isAtSetPosition()
            // Calculate feed forward based on angle to counteract gravity
            double sineScalar = Math.sin(Math.toRadians(getArmDegrees() - Constants.ARM_BALANCE_DEGREES));
            double feedForward = gravityFF * sineScalar;

        //System.out.println(armCanEncoder.getAbsolutePosition().getValueAsDouble());
         // armPidController.setSetpoint(setPosition,
                    //ControlType.kPosition,ClosedLoopSlot.kSlot0, feedForward, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }
    
} 
