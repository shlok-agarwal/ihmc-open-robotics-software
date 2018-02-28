package us.ihmc.exampleSimulations.forceSensorExamples;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FallingBoxRobotController implements RobotController
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final Robot robot;

   private final double dt;

   private final GroundContactPointBasedWrenchCalculator gcpWrenchCalculator;

   private final YoWrench gcpWrench;

   //   private final SimpleCollisionShapeBasedWrenchCalculator csWrenchCalculator;
   //
   //   private final YoWrench cspWrench;

   public FallingBoxRobotController(Robot robot, double dt)
   {
      registry = new YoVariableRegistry(robot.getName() + "_registry");

      this.robot = robot;
      this.dt = dt;

      Joint joint = robot.getJoint("bodyJoint");
      ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<>();
      joint.recursiveGetAllGroundContactPoints(groundContactPoints);
      gcpWrenchCalculator = new GroundContactPointBasedWrenchCalculator(robot.getName() + "_ft", groundContactPoints, joint, new RigidBodyTransform(),
                                                                        registry);

      gcpWrench = new YoWrench(robot.getName() + "_baseWrench", worldFrame, worldFrame, registry);

   }

   @Override
   public void initialize()
   {

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
      gcpWrenchCalculator.calculate();

      DenseMatrix64F wrenchMatrix = gcpWrenchCalculator.getWrench();
      Wrench wrench = new Wrench(worldFrame, worldFrame, wrenchMatrix);

      gcpWrench.set(wrench);
   }

}
