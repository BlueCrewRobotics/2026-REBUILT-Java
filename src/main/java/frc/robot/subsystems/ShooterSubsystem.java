package frc.robot.subsystems;
import java.util.HashMap;
import java.util.Map;
import java.util.Map;
import java.util.HashMap;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AutoShoot;
//shooter1 code
public class ShooterSubsystem extends SubsystemBase {
private final TalonFX motor1 = new TalonFX(Constants.motor1);
private final TalonFX motor2 = new TalonFX(Constants.motor2);
private final TalonFX kickWheelT = new TalonFX(Constants.KICK_WHEEL);
private final SparkMax  kickWheel;
    private TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

public ShooterSubsystem(){

    motor1.clearStickyFaults();
    motor2.clearStickyFaults();
shooterConfig.CurrentLimits.SupplyCurrentLimit=25;
shooterConfig.CurrentLimits.SupplyCurrentLimitEnable=true;

 kickWheel = new SparkMax(Constants.KICK_WHEEL, SparkLowLevel.MotorType.kBrushless);
//motor2.setControl(new Follower(Constants.motor1, null));
}
public Command Shoot(double Speed){
 
    return Commands.runOnce(() -> {motor1.set(Speed); motor2.set(-Speed);});
}
public Command shootBack(double Speed){
     return Commands.runOnce(() -> {motor1.set(-Speed); motor2.set(Speed);});
}

public Command spinMotor(double speed){
return new InstantCommand(() -> motor1.set(speed));
}
public Command stopSpin(){
return Commands.runOnce(()->{ motor1.stopMotor(); motor2.stopMotor();});
}
//motor2 code
public void SpinTheMotor(){
shooterConfig.CurrentLimits.SupplyCurrentLimit=40;
shooterConfig.CurrentLimits.SupplyCurrentLimitEnable=true;
 //motor2.setControl(new Follower(Constants.motor1, null));
}
public Command spinMotor2(double speed ){
return new InstantCommand(() -> motor2.set(-speed));
}
public Command stopSpin2(){
return new InstantCommand(()-> motor2.stopMotor());
}
public Command kick(double speed){
    return new InstantCommand(()-> kickWheel.set(speed));
}
public Command KickOff(){
    return new InstantCommand(()-> kickWheel.stopMotor());
}
public Command kickT(double speed){
    return new InstantCommand(()-> kickWheelT.set(speed));
}
public Command KickOffT(){
    return new InstantCommand(()-> kickWheelT.stopMotor());
}
public Command autoShoot(){
    return new InstantCommand(()-> AutoShoot.newVilocity(VisionPoseEstimator.distance));
}
public Command pulseKick(){
    
    return new SequentialCommandGroup(
        kickT(.5),
        KickOffT()

    );

    //return new InstantCommand(()-> kickT(.1).withTimeout(3).andThen(KickOffT()));
}

public Command shootInAutoPaths(double speed){
    return new ParallelCommandGroup(
        Shoot(speed),
        Commands.sequence(
            Commands.waitSeconds(1.5),
            pulseKick().repeatedly().withTimeout(30)
        )
    );
}
public Command stopAllShooting(){
    return new ParallelCommandGroup(
        stopSpin(),
         KickOffT()
    );
}

private static final Map<Long, Double> distancesToPower;

    static {
        distancesToPower = new HashMap<Long, Double>();;
        distancesToPower.put(-1L, 0.0); // default value for out of range distances
        distancesToPower.put(1L,0.46);
        distancesToPower.put(2L,0.48);
        distancesToPower.put(3L,0.50);
        distancesToPower.put(4L,0.52);
        distancesToPower.put(5L,0.54);
        distancesToPower.put(6L,0.56);
        distancesToPower.put(7L,0.58);
        distancesToPower.put(8L,0.60);
        distancesToPower.put(9L,0.62);
        distancesToPower.put(10L,0.64);
        distancesToPower.put(11L,0.66);
        distancesToPower.put(12L,0.68);
        distancesToPower.put(13L,0.70);
        distancesToPower.put(14L,0.72);
        distancesToPower.put(15L,0.74);
        distancesToPower.put(16L,0.76);
        distancesToPower.put(17L,0.78);
        distancesToPower.put(18L,0.80);
        distancesToPower.put(19L,0.82);
        distancesToPower.put(20L,0.84);
    }



public Double distanceToMotorSpeed(double distance){
    //System.out.println("I have done distance to motor speed");
    long numToGet = Math.round(Units.metersToFeet(distance));
    return distancesToPower.get(numToGet);
}

public Command setMotorSpeedFromDistance(double distance){
    //System.out.println("I have set motor speed");
    return new InstantCommand(() -> Shoot(distanceToMotorSpeed(distance)));
}
}
