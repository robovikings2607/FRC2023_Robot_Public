package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.utilities.RVAprilTagField;

//Class brought to you by Matt M 😼 '23 (and photonlib)
public class PiCameraSubsystem extends SubsystemBase {
    private DriveSubsystem swerveDrive;
    boolean visionOn = true;
    private double allianceSign;
    PhotonPipelineResult target;

    private PhotonCamera camera;
    private Pose2d lastVisionPose;

    private double lastPipelineResultTimestamp = 0;
    // private PhotonPipelineResult result;
    // private PhotonTrackedTarget bestTarget;

    private PhotonPoseEstimator poseEstimator;

    FieldObject2d fieldVisionDetections;
    FieldObject2d fieldVisionPose;
    FieldObject2d fieldNearestNode;

    Transform3d BOT_TO_CAM, CAM_TO_BOT;

    public PiCameraSubsystem(RobotContainer robot, String camName, Transform3d botToCam)
    {
        this.BOT_TO_CAM = botToCam;
        this.CAM_TO_BOT = BOT_TO_CAM.inverse();
        camera = new PhotonCamera(camName);
        this.swerveDrive = robot.swerveDrive;

        poseEstimator = new PhotonPoseEstimator(
            RVAprilTagField.getFieldLayout(), //Field layout
            PoseStrategy.MULTI_TAG_PNP, //Pose strategy
            camera, //Camera object
            BOT_TO_CAM //Robot -> camera Transform3d
        );

        fieldVisionDetections = robot.field.getObject(camera.getName()+"/visionDetections");
        fieldVisionPose = robot.field.getObject(camera.getName()+"/fieldVisionPose");
        fieldNearestNode = robot.field.getObject(camera.getName()+"/fieldNearestNode");
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose(PhotonPipelineResult result)
    {
        var botpose = poseEstimator.update(result);

        if (botpose.isEmpty() || !isOnField(botpose.get().estimatedPose)) 
        {
            return Optional.empty();
        }
        return botpose;
    }

    // public Optional<Pose2d> getNearestNode(){
    //     allianceSign = (DriverStation.getAlliance() == Alliance.Blue) ? -1 : 1;
    //     double autoScoreTolerance = 2.0;
    //     var map = RVAprilTagField.pieceYMap;
    //     var botpose = swerveDrive.getPose().getY();
    //     double resultPose;
    //     if (map.ceilingKey(botpose) == null){
    //         resultPose = map.lowerKey(botpose);
    //     } else if (map.lowerKey(botpose) == null){
    //         resultPose = map.ceilingKey(botpose);
    //     } else if (map.lowerKey(botpose) == null && map.ceilingKey(botpose) == null){
    //         return Optional.empty();
    //     } else {
    //         resultPose = Math.abs((botpose - map.ceilingKey(botpose))) <= Math.abs((botpose - map.lowerKey(botpose))) 
    //             ? map.ceilingKey(botpose) : map.lowerKey(botpose);
    //     }
    //     Pose2d computedPose = new Pose2d(
    //         new Translation2d((6.43)*allianceSign, resultPose), 
    //         new Rotation2d((Math.PI/2) - (Math.PI/2*allianceSign))
    //     );
    //     return (swerveDrive.getPose().getTranslation().getDistance(computedPose.getTranslation()) >= autoScoreTolerance) ?
    //         Optional.empty() : Optional.of(computedPose);
    // }

    private boolean isOnField(Pose3d botpose)
    {
        double maxX = RVAprilTagField.FIELD_LENGTH / 2.0;
        double maxY = RVAprilTagField.FIELD_WIDTH / 2.0;
        return (
            Math.abs(botpose.getX()) < maxX &&
            Math.abs(botpose.getY()) < maxY &&
            Math.abs(botpose.getZ()) < 2.5
        ); //We're within X and Y bounds of the field and we're not flying or underground
    }

    private double getLowestAmbiguity(PhotonPipelineResult pipelineResult)
    {
        if (!pipelineResult.hasTargets()) return -1;
        double lowestAmbiguity = -1;
        var targets = pipelineResult.targets;
        for (int i = 0; i < targets.size(); i++)
        {
            if (targets.get(i).getPoseAmbiguity() == -1) continue;
            if (i == 0 || targets.get(i).getPoseAmbiguity() < lowestAmbiguity)
            {
                lowestAmbiguity = targets.get(i).getPoseAmbiguity();
            }
        }
        return lowestAmbiguity;
    }

    private int getNumberOfTargets(PhotonPipelineResult result){
        return result.targets.size();
    }

    public Pose2d getLastVisionPose() {
        return lastVisionPose;
    }


    void drawTargetsOnField(PhotonPipelineResult result, Optional<EstimatedRobotPose> maybeEstimatedPose)
    {
        if ( maybeEstimatedPose.isEmpty())
        {
            fieldVisionDetections.setPoses(Collections.emptyList());
            fieldVisionPose.setPoses(Collections.emptyList());
            return;
        }
        final var visionPose = maybeEstimatedPose.get().estimatedPose;
        ArrayList<Pose2d> targetPoses = new ArrayList<>(result.targets.size());
        for ( final var target : result.targets )
        {
            var cameraToTarget = target.getBestCameraToTarget();
            targetPoses.add(visionPose.plus(BOT_TO_CAM).plus(cameraToTarget).toPose2d().relativeTo(Constants.LEFT_BOTTOM_CORNER));
        }
        fieldVisionDetections.setPoses(targetPoses);
        fieldVisionPose.setPose(visionPose.toPose2d().relativeTo(Constants.LEFT_BOTTOM_CORNER));
    }

    List<PhotonTrackedTarget> getBadTargets(List<PhotonTrackedTarget> targets) {
        var badTargets = new ArrayList<PhotonTrackedTarget>();
        for ( var target : targets )
        {
            if ( target.getPoseAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD )
            {
                badTargets.add(target);
            }
        }
        return badTargets;
    }

    @Override
    public void periodic()
    {
        // Update node viz.
        var nearest = swerveDrive.getNearestNode();
        if ( nearest.isPresent() )
        {
            fieldNearestNode.setPose(nearest.get().relativeTo(Constants.LEFT_BOTTOM_CORNER));
        }

        // Update vision.
        var pipelineResult = camera.getLatestResult();

        // Only do work on new results.
        if ( pipelineResult.getTimestampSeconds() == lastPipelineResultTimestamp )
        {
            return;
        }

        var badTargets = getBadTargets(pipelineResult.getTargets());
        pipelineResult.targets.removeAll(badTargets);


        var maybeEstimatedPose = getEstimatedRobotPose(pipelineResult);
        SmartDashboard.putNumber("Vision/"+camera.getName()+"/totalTargets", pipelineResult.targets.size());
        SmartDashboard.putNumber("Vision/"+camera.getName()+"/updateDelay", pipelineResult.getTimestampSeconds() - lastPipelineResultTimestamp);
        SmartDashboard.putNumber("Vision/"+camera.getName()+"/badTargets", badTargets.size());
        lastPipelineResultTimestamp = pipelineResult.getTimestampSeconds();
        if (visionOn && maybeEstimatedPose.isPresent()) 
        {
            var estimatedPose = maybeEstimatedPose.get();
            lastVisionPose = estimatedPose.estimatedPose.toPose2d();
            if ( DriverStation.isTeleopEnabled() )
            {
                swerveDrive.addVisionMeasurement(estimatedPose);
            }
        }

        drawTargetsOnField(pipelineResult, maybeEstimatedPose);
    }
}