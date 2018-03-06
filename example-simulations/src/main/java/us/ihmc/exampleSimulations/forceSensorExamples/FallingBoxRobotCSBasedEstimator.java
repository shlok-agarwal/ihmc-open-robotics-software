package us.ihmc.exampleSimulations.forceSensorExamples;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.ContactingExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.CollisionShapeBasedWrenchCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class FallingBoxRobotCSBasedEstimator implements RobotController
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final Robot robot;

   private final double dt;

   private final CollisionShapeBasedWrenchCalculator wrenchCalculator;

   private final YoWrench wrench;

   private final ArrayList<YoFramePoint> externalContactPoints = new ArrayList<YoFramePoint>();
   private final ArrayList<YoFrameVector> externalContactForces = new ArrayList<YoFrameVector>();
   private final ArrayList<YoGraphicPosition> yoExternalContactPointsViz = new ArrayList<>();

   private final YoInteger numberOfContactingPoints;

   public final YoGraphicsListRegistry graphicsRegistry = new YoGraphicsListRegistry();

   public FallingBoxRobotCSBasedEstimator(Robot robot, double dt)
   {
      registry = new YoVariableRegistry("tempName");

      this.robot = robot;
      this.dt = dt;

      wrenchCalculator = new CollisionShapeBasedWrenchCalculator();

      wrench = new YoWrench(robot.getName() + "_wrench_", worldFrame, worldFrame, registry);

      for (int i = 0; i < 50; i++)
      {
         YoFramePoint externalContactPoint = new YoFramePoint("extContactPoint_" + i, worldFrame, registry);
         externalContactPoints.add(externalContactPoint);

         YoFrameVector externalContactForce = new YoFrameVector("extContactForce_" + i, worldFrame, registry);
         externalContactForces.add(externalContactForce);

         YoGraphicPosition yoExternalContactPoint = new YoGraphicPosition("yoextContactPoint_" + i, externalContactPoint, 0.01, YoAppearance.AliceBlue());
         yoExternalContactPointsViz.add(yoExternalContactPoint);

         YoDouble yoContactingTime = new YoDouble("contactingTime_" + i, registry);
      }

      numberOfContactingPoints = new YoInteger("numberOfContactingPoints", registry);

      graphicsRegistry.registerYoGraphics("yopointlist", yoExternalContactPointsViz);
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

      Vector3D force = new Vector3D();

      ArrayList<ContactingExternalForcePoint> allContactingExternalForcePoints = robot.getAllContactingExternalForcePoints();
      numberOfContactingPoints.set(allContactingExternalForcePoints.size());
      for (int i = 0; i < allContactingExternalForcePoints.size(); i++)
      {
         ContactingExternalForcePoint contactingExternalForcePoint = allContactingExternalForcePoints.get(i);
         externalContactPoints.get(i).set(contactingExternalForcePoint.getPositionPoint());

         //         force.setX(contactingExternalForcePoint.getYoImpulse().getX() / dt);
         //         force.setY(contactingExternalForcePoint.getYoImpulse().getY() / dt);
         //         force.setZ(contactingExternalForcePoint.getYoImpulse().getZ() / dt);

         force.set(contactingExternalForcePoint.getYoForce());
         externalContactForces.get(i).set(force);

         wrenchMatrix.set(3, 0, wrenchMatrix.get(3, 0) + force.getX());
         wrenchMatrix.set(4, 0, wrenchMatrix.get(4, 0) + force.getY());
         wrenchMatrix.set(5, 0, wrenchMatrix.get(5, 0) + force.getZ());

      }

      Wrench wrench = new Wrench(worldFrame, worldFrame, wrenchMatrix);

      this.wrench.set(wrench);
   }

}
