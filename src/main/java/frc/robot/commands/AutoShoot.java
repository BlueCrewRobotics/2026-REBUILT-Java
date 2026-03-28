package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionPoseEstimator;

public class AutoShoot { 
    private double distance;
    private double Speed;
    private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    // ---------- MOTOR AND SHOOTER PARAMETERS ----------
    // Example values for Kraken X60/X44 motors
    static final double NO_LOAD_RPM = 6000;    // max RPM at 12V (Kraken X60 example)
    static final double MAX_TORQUE_NM = 0.18;  // max torque in Nm
    static final double GEAR_RATIO = 1;        // direct drive example
    static final double WHEEL_RADIUS_M = 0.05; // shooter wheel radius in meters (5 cm)

    // Physical constants
    static final double GRAVITY = 9.81;        // m/s^2

    // ---------- CONVERSIONS ----------
    public static double rpmToRadPerSec(double rpm) {
        return rpm * 2 * Math.PI / 60.0;
    }

    public static double motorToWheelSpeed(double rpm, double gearRatio) {
        return rpm / gearRatio;
    }

    public static double wheelToBallSpeed(double wheelRpm, double wheelRadius) {
        double omega = rpmToRadPerSec(wheelRpm);
        return omega * wheelRadius;
    }
   public static double[] calculateTrajectory(double distanceM, double initialSpeed, double launchAngleDeg) {
        double angleRad = Math.toRadians(launchAngleDeg);

        // Time of flight (horizontal motion)
        double tFlight = distanceM / (initialSpeed * Math.cos(angleRad));

        // Vertical position at target distance (optional)
        double yFinal = initialSpeed * Math.sin(angleRad) * tFlight - 0.5 * GRAVITY * Math.pow(tFlight, 2);

        return new double[]{tFlight, yFinal};
    }
    public AutoShoot(){

    }
    public double getNewSpeed(){
        return this.Speed;
    }
    public void setNewspeed(double speed){
        this.Speed = speed;
    }

    public double getDistance() {
        return distance;
    }
    public void setDistance(double distamce){
        this.distance = distamce;
    }
    public static double newVilocity(double Distance){
         double selectedMotorRPM = NO_LOAD_RPM; // Kraken X60/X44
        double launchAngleDeg = 45;
        // Calculate ball speed
        double wheelRpm = motorToWheelSpeed(selectedMotorRPM, GEAR_RATIO);
        double ballSpeed = wheelToBallSpeed(wheelRpm, WHEEL_RADIUS_M);
         double[] trajectory = calculateTrajectory(Distance, ballSpeed, launchAngleDeg);
         double timeOfFlight = trajectory[0];
        double finalHeight = trajectory[1];
        return timeOfFlight;
    }
    public Command simulatedShoot(){
        double speed = newVilocity(VisionPoseEstimator.distance);
        return new InstantCommand(()-> shooterSubsystem.Shoot(speed).alongWith(shooterSubsystem.pulseKick()));
    }
}
