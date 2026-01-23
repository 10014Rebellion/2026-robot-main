package frc.robot.systems.vision.object;

import static frc.robot.systems.vision.object.ObjectDetectConstants.kMaxRadiusForCluster;
import static frc.robot.systems.vision.object.ObjectDetectConstants.kMinFuelClusterSize;
import static frc.robot.systems.vision.object.ObjectDetectConstants.kRotationalToleranceDegrees;
import static frc.robot.systems.vision.object.ObjectDetectConstants.kTranslationToleranceMeters;
import static frc.robot.systems.vision.object.ObjectDetectConstants.kDensityHeuristic;
import static frc.robot.systems.vision.object.ObjectDetectConstants.kDistanceHeuristic;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.ml.NearestNeighbour;
import frc.lib.ml.NearestNeighbour.Cluster;
import frc.robot.systems.vision.object.ObjectDetectIOInputsAutoLogged;
import frc.robot.systems.vision.object.ObjectDetectConstants.Camera;

public class ObjectDetect {

    private ObjectDetectIO mFrontRightCamera;
    private ObjectDetectIO mFrontLeftCamera;
    private ObjectDetectIOInputsAutoLogged mFrontRightData;
    private ObjectDetectIOInputsAutoLogged mFrontLeftData;


    public ObjectDetect(ObjectDetectIO[] pCameras) {
        this.mFrontLeftCamera = pCameras[0];
        this.mFrontRightCamera = pCameras[1];
        this.mFrontRightData = new ObjectDetectIOInputsAutoLogged();
        this.mFrontLeftData = new ObjectDetectIOInputsAutoLogged();
    }

    public void periodic(Pose2d pLastRobotPose, Pose2d pSimOdomPose){
        mFrontRightCamera.updateInputs(mFrontRightData, pLastRobotPose, pSimOdomPose);
        Logger.processInputs("Vision/Object-Detection/" + mFrontRightData.iCamName, mFrontRightData);

        mFrontLeftCamera.updateInputs(mFrontLeftData, pLastRobotPose, pSimOdomPose);
        Logger.processInputs("Vision/Object-Detection/" + mFrontLeftData.iCamName, mFrontLeftData);
    }

    public Pose2d[] getObjectPoseForCamera(Camera camera){
        if(camera.equals(Camera.FRONT_RIGHT)){
            return mFrontRightData.iTrackedTargetsPoses;
        }

        else if(camera.equals(Camera.FRONT_LEFT)){
            return mFrontLeftData.iTrackedTargetsPoses;
        }

        else{
            DriverStation.reportError("Obj camera to grab poses from does not exist", true);
            return new Pose2d[0];
        }
    }

    
    public Pose2d[] getObjectPoses(Pose2d pRobotPose){
        Pose2d[] frontLeftPoses = getObjectPoseForCamera(Camera.FRONT_LEFT);
        Pose2d[] frontRightPoses = getObjectPoseForCamera(Camera.FRONT_RIGHT);
        
        // Create a big list of the poses //
        Pose2d[] poses = new Pose2d[frontLeftPoses.length + frontRightPoses.length];
        System.arraycopy(frontLeftPoses, 0, poses, 0, frontLeftPoses.length);
        System.arraycopy(frontRightPoses, 0, poses, frontLeftPoses.length, frontRightPoses.length);
        
        ArrayList<Pose2d> uniquePosesList = new ArrayList<>();

        for (Pose2d pose : poses) {
            boolean isDuplicate = uniquePosesList.stream().anyMatch(u -> areObjectsSame(pose, u));
            if (!isDuplicate){
                uniquePosesList.add(pose);
            }
        }

        for(int i = 0; i < uniquePosesList.size(); i++){
            Pose2d robotRelativePose = uniquePosesList.get(i);
            Translation2d fieldRelativePose = pRobotPose.getTranslation().plus(robotRelativePose.rotateBy(pRobotPose.getRotation()).getTranslation());
            uniquePosesList.set(i, new Pose2d(fieldRelativePose, new Rotation2d()));
        }

        return uniquePosesList.toArray(new Pose2d[0]);

    }

    public Cluster[] getWeightedClusters(Pose2d[] pPoses, Pose2d pRobotPose){
        ArrayList<Pose2d> poses = new ArrayList<>();

        for(Pose2d pose : pPoses){
            poses.add(pose);
        }

        List<Cluster> clusters =  NearestNeighbour.getClusters(poses, kMaxRadiusForCluster, kMinFuelClusterSize);

        for (Cluster cluster : clusters){
            cluster.mWeight = 
                (cluster.mSize * kDensityHeuristic) + 
                (kDistanceHeuristic / (cluster.mCentroid.getTranslation().getDistance(pRobotPose.getTranslation()) + 0.001));
        }

        for (int i = 0; i < clusters.size() - 1; i ++){
            int min_cluster = i;

            for (int j = i + 1; j < clusters.size(); j++) {
                if (clusters.get(j).mWeight > clusters.get(min_cluster).mWeight){
                    min_cluster = j;
                }
            }

            Cluster temporary = clusters.get(i);
            clusters.set(i, clusters.get(min_cluster));
            clusters.set(min_cluster, temporary);
        }

        return clusters.toArray(new Cluster[0]);

    }

    private double atan(Pose2d p) {
        return Math.toDegrees(Math.atan2(p.getY(), p.getX()));
    }

    private boolean isAngleSame(double x, double y, double tolerance){
        double difference = Math.IEEEremainder(x-y, 360.0);
        return Math.abs(difference) <= tolerance;
    }

    private boolean areObjectsSame(Pose2d pose1, Pose2d pose2){
        
        // This boolean expression checks if two poses are 
        return (
            pose1.getTranslation().getDistance(pose2.getTranslation()) <= kTranslationToleranceMeters &&
            isAngleSame(atan(pose1), atan(pose2), kRotationalToleranceDegrees)
        );
    }

    
}
