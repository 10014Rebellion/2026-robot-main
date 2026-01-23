package frc.lib.ml;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NearestNeighbour {
    
    public static class Cluster {

        List<Pose2d> mPoses;
        public Pose2d mCentroid;
        public double mWeight;
        public int mSize;

        public Cluster(List<Pose2d> pPoses){
            this.mPoses = pPoses;

            // No point of running extra computations unless neccessary //
            if (!mPoses.isEmpty()){
                mCentroid = calculateCentroid();
                mSize = mPoses.size();
            }

            this.mWeight = Double.MIN_VALUE;
        }


        public Pose2d calculateCentroid(){
            
            
            if(mPoses.size() == 0){
                return new Pose2d();
            }

            Pose2d centroid;
            double cX = 0;
            double cY = 0;

            for(Pose2d pose : mPoses){
                cX += pose.getX();
                cY += pose.getY();
            }

            centroid = new Pose2d(new Translation2d(cX / mPoses.size(), cY / mPoses.size()), new Rotation2d());

            return centroid;
        }

        public double getSize(){
            return mSize;
        }

        public void addPose(Pose2d pose){
            if (mPoses.isEmpty()){
                mCentroid = new Pose2d(pose.getTranslation(), new Rotation2d());
            }


            // Mathematical Identity utilized !! //
            // new centroid.X = (old centroid.X * size) + new node.X / old centroid.N + 1
            // new centroid.Y = (old centroid.Y * size) + new node.Y / old centroid.N + 1
            else{
                int curSize = mPoses.size();

                double cX = ((mCentroid.getX() * curSize ) + pose.getX()) / (curSize + 1);
                double cY = ((mCentroid.getY() * curSize ) + pose.getY()) / (curSize + 1);
                mCentroid = new Pose2d(new Translation2d(cX, cY), new Rotation2d());
            }

            mPoses.add(pose);
            mSize ++;
        }

        public void setWeight(double pWeight){
            this.mWeight = pWeight;
        }
    }

    public static List<Cluster> getClusters(List<Pose2d> pPoses, double pMaxRadius, int pMinFuelSize){
        List<Cluster> clusters = new ArrayList<>();
        Set<Integer> visited = new HashSet<>();

        for (int i = 0; i < pPoses.size(); i++){

            if (visited.contains(i)){
                continue;
            }

            // Create a cluster based with an empty list //
            Cluster cluster = new Cluster(new ArrayList<Pose2d>());

            // This line ensures we recursively go through and check for poses that are potential poses for the cluster//
            expandCluster(pPoses, i, visited, cluster, pMaxRadius);

            // Only want to add a cluster if the amount of fuel is greater than a set threshold we determine //
            if (cluster.getSize() >= pMinFuelSize){
                clusters.add(cluster);
            }
        }

        return clusters;
    }
    
    // Will recurisvely go through a list of poses and expand the cluser to have more fuels // 
    private static void expandCluster(List<Pose2d> pPoses, int pIndex, Set<Integer> pVisited, Cluster pCluster, double pMaxRadius){

        // This is the base conddition to stop infnite recursion //
        if (pVisited.contains(pIndex)){
            return;
        }

        // Update some vital variables //
        pVisited.add(pIndex);
        Pose2d currentPose = pPoses.get(pIndex);
        pCluster.addPose(currentPose);

        for (int i = 0; i < pPoses.size(); i++){

            if (pVisited.contains(i)){
                continue;
            }

            double distance = euclideanDistance(currentPose, pPoses.get(i));

            if(distance <= pMaxRadius){
                expandCluster(pPoses, i, pVisited, pCluster, pMaxRadius);
            }
        }
    }

    // Metric to decide the distance between two poses // 
    private static double euclideanDistance(Pose2d initial, Pose2d to){
        return Math.sqrt(
            Math.pow((to.getX() - initial.getX()), 2) +
            Math.pow((to.getY() - initial.getY()), 2));
    }
}
