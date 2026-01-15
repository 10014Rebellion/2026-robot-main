package frc.lib.sorting;

import edu.wpi.first.math.geometry.Pose2d;

class QuickSort {

    // partition function
    static int partition(Pose2d[] arr, int low, int high) {
        
        // choose the pivot
        double pivot = convertToDistance(arr[high]);
        
        // index of smaller element and indicates 
        // the right position of pivot found so far
        int i = low - 1;

        // traverse arr[low..high] and move all smaller
        // elements to the left side. Elements from low to 
        // i are smaller after every iteration
        for (int j = low; j <= high - 1; j++) {
            if (convertToDistance(arr[j]) < pivot) {
                i++;
                swap(arr, i, j);
            }
        }
        
        // Move pivot after smaller elements and
        // return its position
        swap(arr, i + 1, high);  
        return i + 1;
    }

    // swap function
    static void swap(Pose2d[] arr, int i, int j) {
        Pose2d temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }

    // the QuickSort function implementation
    static void quickSort(Pose2d[] arr, int low, int high) {
        if (low < high) {
            
            // pi is the partition return index of pivot
            int pi = partition(arr, low, high);

            // recursion calls for smaller elements
            // and greater or equals elements
            quickSort(arr, low, pi - 1);
            quickSort(arr, pi + 1, high);
        }
    }

    private static double convertToDistance(Pose2d pose){
        return pose.getTranslation().getNorm();
    }

    public static void sortPose2d(Pose2d[] args) {
      
        int n = args.length;
        quickSort(args, 0, n - 1);
    }
}