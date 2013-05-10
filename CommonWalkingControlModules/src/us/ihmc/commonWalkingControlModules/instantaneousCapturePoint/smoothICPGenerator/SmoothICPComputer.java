package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class SmoothICPComputer
{
// private final ArrayList<Point3d> footLocations = new ArrayList<Point3d>();
   private final ArrayList<Point3d> constantCentersOfPressure = new ArrayList<Point3d>();

   private final int maxNumberOfConsideredFootsteps;
   private final double doubleSupportFirstStepFraction;

   private boolean isDoubleSupport;
   private double initialTime;
   private boolean comeToStop;
   
   private double singleSupportDuration;
   private double doubleSupportDuration;
   private double doubleSupportInitialTransferDuration;

   private final NewDoubleSupportICPComputer newDoubleSupportICPComputer;
   private Point3d[] icpCornerPoints;

   private Point3d constantCenterOfPressure;

   private Point3d singleSupportStartICP;
   private final Point3d
      doubleSupportStartICP = new Point3d(), doubleSupportEndICP = new Point3d();
   private final Vector3d
      doubleSupportStartICPVelocity = new Vector3d(), doubleSupportEndICPVelocity = new Vector3d();

   private final DenseMatrix64F doubleSupportParameterMatrix = new DenseMatrix64F(3, 4);

   public SmoothICPComputer(double doubleSupportFirstStepFraction, int maxNumberOfConsideredFootsteps)
   {
      this.doubleSupportFirstStepFraction = doubleSupportFirstStepFraction;
      this.maxNumberOfConsideredFootsteps = maxNumberOfConsideredFootsteps;

      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         constantCentersOfPressure.add(new Point3d());
      }

      newDoubleSupportICPComputer = new NewDoubleSupportICPComputer();
   }

   public void initializeSingleSupport(ArrayList<Point3d> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime, boolean stopIfReachedEnd)
   {
      boolean isInitialTransfer = false;
      comeToStop = false;
      
      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer, stopIfReachedEnd);

      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportDuration = doubleSupportDuration;
      this.doubleSupportInitialTransferDuration = Double.NaN;

      this.isDoubleSupport = false;

      this.initialTime = initialTime;

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressure, steppingDuration, omega0);

      Point3d cornerPoint0 = icpCornerPoints[0];
      Point3d supportFoot = constantCentersOfPressure.get(0);

      singleSupportStartICP = newDoubleSupportICPComputer.computeSingleSupportStartICP(supportFoot, cornerPoint0, doubleSupportDuration,
              doubleSupportFirstStepFraction, omega0);

   }

   public List<Point3d> getConstantCentersOfPressure()
   {
      return constantCentersOfPressure;
   }

   private static void computeConstantCentersOfPressure(ArrayList<Point3d> constantCentersOfPressureToModify, ArrayList<Point3d> footLocations,
           int maxNumberOfConsideredFootsteps, boolean isInitialTransfer, boolean stopIfReachedEnd)
   {
      boolean putFirstCenterOfPressureInMiddle = isInitialTransfer;
      boolean willReachTheEnd = footLocations.size() <= maxNumberOfConsideredFootsteps;
      boolean putTheLastNCentersOfPressureInMiddle = stopIfReachedEnd && willReachTheEnd;

      int numberInFootlist = footLocations.size();
      Point3d positionToHoldAt = new Point3d();

      for (int i = 0; i < maxNumberOfConsideredFootsteps; i++)
      {
         Point3d centerOfPressureLocation = constantCentersOfPressureToModify.get(i);

         if (i == 0)
         {
            if (putFirstCenterOfPressureInMiddle)
            {
               centerOfPressureLocation.set(footLocations.get(0));
               centerOfPressureLocation.add(footLocations.get(1));
               centerOfPressureLocation.scale(0.5);
            }
            else
            {
               centerOfPressureLocation.set(footLocations.get(0));
            }
         }

         else if (i < numberInFootlist - 1)
         {
            centerOfPressureLocation.set(footLocations.get(i));
         }

         else
         {
            if (i == numberInFootlist - 1)
            {
               if (putTheLastNCentersOfPressureInMiddle)
               {
                  positionToHoldAt.set(footLocations.get(i));
                  positionToHoldAt.add(footLocations.get(i - 1));
                  positionToHoldAt.scale(0.5);
               }
               else
               {
                  positionToHoldAt.set(footLocations.get(i));
               }
            }

            centerOfPressureLocation.set(positionToHoldAt);
         }
      }
   }

   public void initializeDoubleSupportInitialTransfer(ArrayList<Point3d> footLocationList, Point3d initialICPPosition, double singleSupportDuration, double doubleSupportDuration,
           double doubleSupportInitialTransferDuration, double omega0, double initialTime, boolean stopIfReachedEnd)
   {
      boolean isInitialTransfer = true;
      comeToStop = footLocationList.size() < 3;
      
      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer, stopIfReachedEnd);

      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportDuration = doubleSupportDuration;
      this.doubleSupportInitialTransferDuration = doubleSupportInitialTransferDuration;

      this.isDoubleSupport = true;
      this.initialTime = initialTime;

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressure, steppingDuration, omega0);

      Point3d cornerPoint1 = icpCornerPoints[1];
      Point3d transferToFoot = constantCentersOfPressure.get(1);

      doubleSupportStartICP.set(constantCentersOfPressure.get(0));
      doubleSupportStartICPVelocity.set(0.0, 0.0, 0.0);

      newDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
              doubleSupportDuration, doubleSupportFirstStepFraction, omega0);

      DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportInitialTransferDuration, doubleSupportStartICP,
              doubleSupportStartICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);
   }


   public void initializeDoubleSupport(ArrayList<Point3d> footLocationList, double singleSupportDuration, double doubleSupportDuration, double omega0,
           double initialTime, boolean stopIfReachedEnd)
   {
      boolean isInitialTransfer = false;
      comeToStop = footLocationList.size() < 3;

      computeConstantCentersOfPressure(constantCentersOfPressure, footLocationList, maxNumberOfConsideredFootsteps, isInitialTransfer, stopIfReachedEnd);

      this.singleSupportDuration = singleSupportDuration;
      this.doubleSupportDuration = doubleSupportDuration;
      this.doubleSupportInitialTransferDuration = Double.NaN;

      this.isDoubleSupport = true;

      this.initialTime = initialTime;

      int numberOfCornerPoints = maxNumberOfConsideredFootsteps - 1;
      double steppingDuration = singleSupportDuration + doubleSupportDuration;
      icpCornerPoints = newDoubleSupportICPComputer.computeICPCornerPoints(numberOfCornerPoints, constantCentersOfPressure, steppingDuration, omega0);

      Point3d cornerPoint0 = icpCornerPoints[0];
      Point3d cornerPoint1 = icpCornerPoints[1];
      Point3d transferFromFoot = constantCentersOfPressure.get(0);
      Point3d transferToFoot = constantCentersOfPressure.get(1);

      newDoubleSupportICPComputer.computeSingleSupportEndICPAndVelocity(doubleSupportStartICP, doubleSupportStartICPVelocity, transferFromFoot, cornerPoint0,
              doubleSupportDuration, doubleSupportFirstStepFraction, singleSupportDuration, omega0);
      newDoubleSupportICPComputer.computeSingleSupportStartICPAndVelocity(doubleSupportEndICP, doubleSupportEndICPVelocity, transferToFoot, cornerPoint1,
              doubleSupportDuration, doubleSupportFirstStepFraction, omega0);

      DoubleSupportICPComputer.computeThirdOrderPolynomialParameterMatrix(doubleSupportParameterMatrix, doubleSupportDuration, doubleSupportStartICP,
              doubleSupportStartICPVelocity, doubleSupportEndICP, doubleSupportEndICPVelocity);

   }


   public Point3d getDoubleSupportStartICP()
   {
      return doubleSupportStartICP;
   }

   public Point3d getDoubleSupportEndICP()
   {
      return doubleSupportEndICP;
   }


   public Vector3d getDoubleSupportStartICPVelocity()
   {
      return doubleSupportStartICPVelocity;
   }

   public Vector3d getDoubleSupportEndICPVelocity()
   {
      return doubleSupportEndICPVelocity;
   }

   public Point3d[] getICPCornerPoints()
   {
      return icpCornerPoints;
   }


   public void getICPPositionAndVelocity(Point3d icpPostionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double omega0, double time)
   {
      double timeInState = time - initialTime;

      if (isDoubleSupport)
         getICPPositionAndVelocityDoubleSupport(icpPostionToPack, icpVelocityToPack, ecmpToPack, omega0, timeInState);
      else
         getICPPositionAndVelocitySingleSupport(icpPostionToPack, icpVelocityToPack, ecmpToPack, omega0, timeInState);
   }

   private void getICPPositionAndVelocitySingleSupport(Point3d icpPositionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double omega0,
           double timeInState)
   {
      constantCenterOfPressure = constantCentersOfPressure.get(0);
      newDoubleSupportICPComputer.computeSingleSupportICPPositionAndVelocity(icpPositionToPack, icpVelocityToPack, constantCenterOfPressure,
              singleSupportStartICP, omega0, timeInState);

      ecmpToPack.set(constantCenterOfPressure);
   }

   private void getICPPositionAndVelocityDoubleSupport(Point3d icpPostionToPack, Vector3d icpVelocityToPack, Point3d ecmpToPack, double omega0,
           double timeInState)
   {
      if (comeToStop && (timeInState > doubleSupportDuration))
      {
         icpPostionToPack.set(doubleSupportEndICP);
         icpVelocityToPack.set(0.0, 0.0, 0.0);
         ecmpToPack.set(icpPostionToPack);
      }
      
      else
      {
         double timePow3 = Math.pow(timeInState, 3.0);
         double timePow2 = Math.pow(timeInState, 2.0);
         Vector3d tempVector = new Vector3d();

         DenseMatrix64F dcmPositionTimeVector = new DenseMatrix64F(4, 1, true, timePow3, timePow2, timeInState, 1);
         DenseMatrix64F dcmVelocityTimeVector = new DenseMatrix64F(4, 1, true, 3 * timePow2, 2 * timeInState, 1, 0);

         multiplyMatricesAndPutInPoint3d(icpPostionToPack, doubleSupportParameterMatrix, dcmPositionTimeVector);
         multiplyMatricesAndPutInPoint3d(icpVelocityToPack, doubleSupportParameterMatrix, dcmVelocityTimeVector);
         tempVector.set(icpVelocityToPack);
         tempVector.scale(-1.0 / omega0);
         ecmpToPack.set(icpPostionToPack);
         ecmpToPack.add(tempVector);
      }
   }

   private void multiplyMatricesAndPutInPoint3d(Tuple3d tuple3dToPack, DenseMatrix64F matrixOne, DenseMatrix64F matrixTwo)
   {
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1);
      CommonOps.mult(matrixOne, matrixTwo, tempMatrix);

      if (tempMatrix.getNumCols() != 1)
         throw new RuntimeException("tempMatrix.getNumCols() != 1");
      if (tempMatrix.getNumRows() != 3)
         throw new RuntimeException("tempMatrix.getNumRows() != 3");

      tuple3dToPack.setX(tempMatrix.get(0, 0));
      tuple3dToPack.setY(tempMatrix.get(1, 0));
      tuple3dToPack.setZ(tempMatrix.get(2, 0));
   }



}
