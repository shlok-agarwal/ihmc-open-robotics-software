package us.ihmc.exampleSimulations.forceSensorExamples;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.CollisionShapeBasedWrenchCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FallingBoxRobotCSBasedEstimator implements RobotController
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final YoVariableRegistry registry;
   
   private final Robot robot;

   private final double dt;
   
   private final CollisionShapeBasedWrenchCalculator wrenchCalculator;
   
   private final YoWrench wrench;

   public FallingBoxRobotCSBasedEstimator(Robot robot, double dt)
   {
      registry = new YoVariableRegistry("tempName");
      
      this.robot = robot;
      this.dt = dt;
      
      wrenchCalculator = new CollisionShapeBasedWrenchCalculator();
  
      wrench = new YoWrench(robot.getName() + "_wrench", worldFrame, worldFrame, registry);
      
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
      wrenchCalculator.calculate();

      DenseMatrix64F wrenchMatrix = wrenchCalculator.getWrench();
      Wrench wrench = new Wrench(worldFrame, worldFrame, wrenchMatrix);

      this.wrench.set(wrench);
   }

}
