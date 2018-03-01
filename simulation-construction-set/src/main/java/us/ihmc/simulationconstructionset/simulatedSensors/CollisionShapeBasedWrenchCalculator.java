package us.ihmc.simulationconstructionset.simulatedSensors;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Joint;

public class CollisionShapeBasedWrenchCalculator implements WrenchCalculatorInterface
{
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
      // TODO Auto-generated method stub
      
   }

   @Override
   public DenseMatrix64F getWrench()
   {
      // TODO Auto-generated method stub
      return null;
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
