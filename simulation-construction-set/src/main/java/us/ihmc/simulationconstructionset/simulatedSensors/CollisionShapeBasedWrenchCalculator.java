package us.ihmc.simulationconstructionset.simulatedSensors;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.Joint;

public class CollisionShapeBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   
   /**
    * wrenchMatrix
    * 0 : tau x
    * 1 : tau y
    * 2 : tau z
    * 3 : f x
    * 4 : f y
    * 5 : f z
    */
      
   public CollisionShapeBasedWrenchCalculator()
   {
      
   }

   @Override
   public String getName()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void getTransformToParentJoint(RigidBodyTransform transformToPack)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void calculate()
   {
      wrenchMatrix.set(0, 0, 100);
      wrenchMatrix.set(5, 0, 200);
      
   }

   @Override
   public DenseMatrix64F getWrench()
   {
      return wrenchMatrix;
   }

   @Override
   public Joint getJoint()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void corruptWrenchElement(int row, double value)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setDoWrenchCorruption(boolean value)
   {
      // TODO Auto-generated method stub
      
   }

}
