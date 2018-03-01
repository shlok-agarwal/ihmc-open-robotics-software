package us.ihmc.simulationconstructionset.simulatedSensors;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimpleCollisionShapeBasedWrenchCalculator implements WrenchCalculatorInterface
{

   public SimpleCollisionShapeBasedWrenchCalculator(String forceSensorName, List<GroundContactPoint> contactPoints, Joint forceTorqueSensorJoint,
                                                    RigidBodyTransform transformToParentJoint, YoVariableRegistry registry)
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
