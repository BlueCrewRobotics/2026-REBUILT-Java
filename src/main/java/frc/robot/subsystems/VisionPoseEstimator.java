/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.subsystems;


 import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
 import edu.wpi.first.math.geometry.Pose2d;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
 import static frc.robot.Constants.PhotonVision.MIN_DISTANCE_TO_TAG_IN_METERS;
 import static frc.robot.Constants.PhotonVision.MAX_DISTANCE_TO_TAG_IN_METERS;
  import static frc.robot.Constants.PhotonVision.TAGS_TO_SHOOT;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
 import edu.wpi.first.math.kinematics.SwerveModulePosition;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.AutoShoot;

import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.targeting.PhotonPipelineResult;
 import frc.robot.generated.TunerConstants;
 public final class VisionPoseEstimator {
 public double speed;
 public static double distance;
     private static final Matrix<N3, N1> multiTagStdDevs = Constants.PhotonVision.multiTagStdDevs;
         private static final Object singleTagStdDevs = Constants.PhotonVision.singleTagStdDevs;
              private final PhotonCamera camera1;
               private final PhotonCamera camera2;
               private PhotonCamera currentCamera;
               private PhotonPoseEstimator photonEstimator1;
               private PhotonPoseEstimator photonEstimator2;
               private PhotonPoseEstimator currentPhotonEstimator;
           
               private double lastEstTimestamp = 0;
           
               private final SwerveDrivePoseEstimator swervePoseEstimator;
           
               private static VisionPoseEstimator instance;
           
               VisionPoseEstimator() {
                   camera1 = VisionModule.getInstance().getAprilTagsFrontRightCamera();
                   camera2 = VisionModule.getInstance().getAprilTagsRearLeftCamera();
                   currentCamera = camera1;
           
                   photonEstimator1 = VisionModule.getInstance().getPhotonEstimatorFrontRight();
                   photonEstimator2 = VisionModule.getInstance().getPhotonEstimatorRearLeft();
                   currentPhotonEstimator = photonEstimator1;
           
                   var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); // TODO: Tune the standard deviations
                   var visionStdDevs = VecBuilder.fill(1, 1, 1);
           
                   swervePoseEstimator = new SwerveDrivePoseEstimator(
                                   //new SwerveDriveKinematics(),
                                   new SwerveDriveKinematics(
    new Translation2d(TunerConstants.FrontLeft.LocationX,  TunerConstants.FrontLeft.LocationY),
    new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
    new Translation2d(TunerConstants.BackLeft.LocationX,   TunerConstants.BackLeft.LocationY),
    new Translation2d(TunerConstants.BackRight.LocationX,  TunerConstants.BackRight.LocationY)
),
                                   new Rotation2d(),
                                   new SwerveModulePosition[] {
                                           new SwerveModulePosition(),
                                           new SwerveModulePosition(),
                                           new SwerveModulePosition(),
                                           new SwerveModulePosition()
                                   },
                                   new Pose2d(),
                                   stateStdDevs,
                                   visionStdDevs);
               }
           
               public static synchronized VisionPoseEstimator getInstance() {
                   if (instance == null) {
                       instance = new VisionPoseEstimator();
                   }
                   return instance;
               }
           
               /*
               private void setCamera(PhotonCamera camera) {
                   currentCamera = camera;
               }
               
               private void setPhotonEstimator(Transform3d robotToCamera) {
                   photonEstimator = new PhotonPoseEstimator(
                           tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, currentCamera, robotToCamera
                   );
               }
               */
           
               private void useBestCamera() {
                   //var cam1Targets = camera1.getLatestResult().getTargets();
                  // var cam2Targets = camera2.getLatestResult().getTargets();
                   var cam1Targets = camera1.getAllUnreadResults().get(camera1.getAllUnreadResults().size()-1).getTargets();
                   var cam2Targets = camera2.getAllUnreadResults().get(camera2.getAllUnreadResults().size()-1).getTargets();
                   int cam1numTags = 0;
                   int cam2numTags = 0;
                   double cam1AvgDist = 0;
                   double cam2AvgDist = 0;
           
                   Transform3d robotToCamera;
           
                   for (var tgt : cam1Targets) {
                       var tagPose = currentPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                       if (tagPose.isEmpty()) continue;
                       cam1numTags++;
                       cam1AvgDist +=
                               tagPose.get().toPose2d().getTranslation().getDistance(swervePoseEstimator.getEstimatedPosition().getTranslation());
                   }
                   cam1AvgDist /= cam1numTags;
                   for (var tgt : cam2Targets) {
                       var tagPose = currentPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                       if (tagPose.isEmpty()) continue;
                       cam2numTags++;
                       cam2AvgDist +=
                               tagPose.get().toPose2d().getTranslation().getDistance(swervePoseEstimator.getEstimatedPosition().getTranslation());
                   }
                   cam2AvgDist /= cam2numTags;
           
                   if (cam2numTags > cam1numTags) {
                       currentCamera = camera2;
                       currentPhotonEstimator = photonEstimator2;
                   } else if (cam2AvgDist < cam1AvgDist && cam2numTags == cam1numTags) {
                       currentCamera = camera2;
                       currentPhotonEstimator = photonEstimator2;
                   } else  {
                       currentCamera = camera1;
                       currentPhotonEstimator = photonEstimator1;
                   }
               }

                public PhotonPipelineResult getLatestResult() {
                    //return currentCamera.getLatestResult();
                   var results = currentCamera.getAllUnreadResults();
                    return results.get(results.size()-1);
               }

           
               /**
                * The latest estimated robot pose on the field from vision data. This may be empty. This should
                * only be called once per loop.
                *
                * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
                *     used for estimation.
                */
               public Optional<EstimatedRobotPose> getEstimatedVisionGlobalPose() {
                   useBestCamera();
                   var visionEst = currentPhotonEstimator.update(getLatestResult());
                //  double latestTimestamp = ((PhotonPipelineResult) currentCamera.getAllUnreadResults()).getTimestampSeconds();
                // double latestTimestamp = currentCamera.getAllUnreadResults().get(currentCamera.getAllUnreadResults().size()-1).getTimestampSeconds();
                    double latestTimestamp = currentCamera.getAllUnreadResults().get(currentCamera.getAllUnreadResults().size()-1).getTimestampSeconds();
                   boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
                   if (newResult) lastEstTimestamp = latestTimestamp;
                   return (Optional<EstimatedRobotPose>) visionEst;
               }

               public double getTargetRotationValue(Rotation2d CurrentYaw){
                currentCamera = camera2;
                var result = getLatestResult();
                double targetYaw = CurrentYaw.getDegrees();
                double newYaw;
                if (!result.getTargets().isEmpty()) {
                    if (result.hasTargets()) {
                        newYaw = result.getBestTarget().getYaw();
                        if (newYaw > .1 || newYaw < -.1) {
                            if (newYaw > .1) {
                               targetYaw += newYaw; 
                            }
                            else {
                                targetYaw -= newYaw;
                            }
                        }
                    }
                }
                return targetYaw;
               }
           
               /**
                * The standard deviations of the estimated pose from {@link #getEstimatedVisionGlobalPose()}, for use
                * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
                * This should only be used when there are targets visible.
                *
                * @param estimatedPose The estimated pose to guess standard deviations for.
                */
                public Matrix<N3, N1> getVisionEstimationStdDevs(Pose2d estimatedPose) {
                   if (DriverStation.isDisabled()) {
                       return VecBuilder.fill(0.001d, 0.001d, 0.001d);
                   }
             
                    Matrix<N3, N1> estStdDevs = (Matrix<N3, N1>) singleTagStdDevs;
                    var targets = getLatestResult().getTargets();
                    int numTags = 0;
                    double avgDist = 0;
                    for (var tgt : targets) {
                        var tagPose = currentPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                        if (tagPose.isEmpty()) continue;
                        numTags++;
                        avgDist +=
                                tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
                    }
                    if (numTags == 0) return (Matrix<N3, N1>) estStdDevs;
                    avgDist /= numTags;
                            
                    // Decrease std devs if multiple targets are visible
                    if (numTags > 1) estStdDevs = multiTagStdDevs;
                    // Increase std devs based on (average) distance
                    if (numTags == 1 && avgDist > 4)
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            
                    return (Matrix<N3, N1>) estStdDevs;
                }
 
     /**
      * @return The {@link Pose2d} of the robot according to the {@link SwerveDrivePoseEstimator}
      */
     public synchronized Pose2d getPose() {
         return swervePoseEstimator.getEstimatedPosition();
     }
 
     /**
      * @param pose The {@link Pose2d} to set the {@link SwerveDrivePoseEstimator} to
      */
     public synchronized void setPose(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions, Pose2d pose) {
         swervePoseEstimator.resetPosition(gyroYaw, modulePositions, pose);
     }
 
     /**
      * Update the {@link SwerveDrivePoseEstimator} with the yaw and module positions
      *
      * @param gyroYaw The yaw of the gyro as a {@link Rotation2d}
      * @param modulePositions The {@link SwerveModulePosition} of each swerve module in an array
      */
     public synchronized void updateSwerveEstimator(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
         swervePoseEstimator.update(gyroYaw, modulePositions);
     }
 
     public synchronized void updateWithVision() {
         var visionEst = getEstimatedVisionGlobalPose();
         visionEst.ifPresent(
                 est -> {
                     var estPose = est.estimatedPose.toPose2d();
                     // Change our trust in the measurement based on the tags we can see
                     var estStdDevs = getVisionEstimationStdDevs(estPose);
 
                     swervePoseEstimator.addVisionMeasurement(
                             est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                 });
     }

     public double getDistanceToTarget() {
        var result1 = camera1.getLatestResult();
        if (result1.hasTargets()) {
            for (var target : result1.getTargets()) {
                // Check if ID is in your TAGS_TO_SHOOT array
                for (int id : TAGS_TO_SHOOT) {
                    if (target.getFiducialId() == id) {
                        distance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
                        if (distance < 1) {
                            return -1; // Return -1 if the target is too close, as it's likely a false positive
                        }
                           return distance; 
                    }
                }
            }
        }
        var result2 = camera2.getLatestResult();
        if (result2.hasTargets()) {
            for (var target : result2.getTargets()) {
                // Check if ID is in your TAGS_TO_SHOOT array
                for (int id : TAGS_TO_SHOOT) {
                    if (target.getFiducialId() == id) {
                        distance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
                        if (distance < 1) {
                            return -1; // Return -1 if the target is too close, as it's likely a false positive
                        }
                           return distance; 
                    }
                }
            }
        }
        //System.out.println("to your old lover");
        return -1;
    }

     public boolean isAnyCameraInRange() {
        // Run both checks and return true if either camera sees a target in range
        distance = getDistanceToTarget();
        return isItInRange(distance);
    }

    // Helper method so you don't repeat the same code twice
    private boolean isItInRange(double distance) {
        if (distance >= MIN_DISTANCE_TO_TAG_IN_METERS && distance <= MAX_DISTANCE_TO_TAG_IN_METERS) {
            return true; 
        }
        //System.out.println("to your old lover");
        return false;
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



public double distanceToMotorSpeed(){
    //System.out.println("I have done distance to motor speed");
    try {
    long numToGet = Math.round(Units.metersToFeet(distance));
    double valueToReturn = distancesToPower.get(numToGet);
    return valueToReturn;
    } catch (NullPointerException e ){
        return 0.0;
    }
}
}
