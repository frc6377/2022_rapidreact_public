package frc.robot.common.utilities;

// Java program to calculate
// Simple Moving Average

//Original From here: https://www.geeksforgeeks.org/program-find-simple-moving-average/ (Does not remove the biggest and smallest)
  
public class MovingAverage {
      
    private static double[] DataSet = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
   public MovingAverage (){
   }

    private static int iterate = 0;

    // function to add new data in the
    // list and update the sum so that
    // we get the new mean
    public void addData(double num)
    {
        DataSet[iterate%10] = num;
        iterate++;
    }

    public double computeSDev(){
      Double mean = getMean();
      Double squareDistanceFromMean = 0.0;

      for (Double i : DataSet) {
        squareDistanceFromMean += Math.pow(mean - i, 2);
      }

      squareDistanceFromMean = squareDistanceFromMean / 10;


      squareDistanceFromMean = Math.sqrt(squareDistanceFromMean);

      return squareDistanceFromMean;

    }
  
    // function to calculate mean
    public double getMean()
    {
      double sum = 0;
      for(Double d : DataSet)
          sum += d;
      return sum/10;
    }

    public boolean isOutlier(double num){
      double mean = getMean();
      double stDev = computeSDev();
      return num > (mean + (2*stDev)) || num < (mean - (2*stDev));
    }
}