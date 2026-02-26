package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotState;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmUpwardsHighGravityPID;;
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
//motor
    ArmMotor = new SparkMax(Constants.ARM_MOTOR,SparkLowLevel.MotorType.kBrushless);
//PID controloer 
    armPidController = ArmMotor.getClosedLoopController();
// encodoer 
    armEncoder = (SparkRelativeEncoder) ArmMotor.getEncoder();
//geting position of the encoder 
    setPosition = armEncoder.getPosition();
    // figuer out how to make this go to the corect posithion me cant 
    armEncoder.setPosition(0);
    }
/* 
    public Command armToHbLevel(){
        return new InstantCommand(() -> setPosition(Constants.ARM_AT_HUB_POSITION));
    }
    public Command armToIntakePosition(){
        return new InstantCommand(() -> setPosition = Constants.ARM_AT_INTAKE_POSITION);
    }
    */
   
} 
