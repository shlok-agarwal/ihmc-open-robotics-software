package us.ihmc.exampleSimulations.forceSensorExamples;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.CollisionShapeBasedWrenchCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FallingBoxRobotCSBasedEstimator implements RobotController
{
   private final YoVariableRegistry registry;
   
   private final CollisionShapeBasedWrenchCalculator wrenchCalculator;

   public FallingBoxRobotCSBasedEstimator(Robot robot, double dt)
   {
      registry = new YoVariableRegistry("tempName");
      
      wrenchCalculator = new CollisionShapeBasedWrenchCalculator();
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub

   }

}
