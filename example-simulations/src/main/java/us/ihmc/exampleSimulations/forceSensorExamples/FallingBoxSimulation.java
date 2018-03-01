package us.ihmc.exampleSimulations.forceSensorExamples;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionVisualizer;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class FallingBoxSimulation
{
   static double dt = 0.001;

   static double mass = 99.0;
   static double heightOfTwoRobots = 1.0;
   static double gapOfTwoRobots = 1.0;

   static Point3D initialPositionOne = new Point3D(0.0, -0.5 * gapOfTwoRobots, heightOfTwoRobots);
   static Point3D initialPositionTwo = new Point3D(0.0, 0.5 * gapOfTwoRobots, heightOfTwoRobots);
   static Vector3D initialOrientationAxisAngle = new Vector3D(0.0, Math.PI * 0.0, 0.0);
   static Quaternion initialOrientation = new Quaternion(initialOrientationAxisAngle);

   static Box3D rootBox = new Box3D(0.05, 0.05, 0.1);
   static Box3D bodyBox = new Box3D(0.5, 0.3, 0.15);

   /**
    * this test is to compare force result between case that robot has ground contact point and case that robot has collision shape.
    */
   public FallingBoxSimulation()
   {
      // robot with ground contact point.
      FallingBoxRobotDescription descriptionWithGCP = new FallingBoxRobotDescription("robotWithGCP", rootBox, bodyBox, mass, true);
      Robot robotWithGCP = new RobotFromDescription(descriptionWithGCP);
      FloatingJoint floatingJointOne = (FloatingJoint) robotWithGCP.getRootJoints().get(0);
      floatingJointOne.setPosition(initialPositionOne);
      floatingJointOne.setQuaternion(initialOrientation);
      GroundContactModel groundModel = new LinearGroundContactModel(robotWithGCP, 1422, 150.6, 50.0, 1000.0, robotWithGCP.getRobotsYoVariableRegistry());
      robotWithGCP.setGroundContactModel(groundModel);
      FallingBoxRobotGCPBasedEstimator gcpBasedEstimator = new FallingBoxRobotGCPBasedEstimator(robotWithGCP, dt);
      robotWithGCP.setController(gcpBasedEstimator);

      //robot with collision shape.
      FallingBoxRobotDescription descriptionWithCS = new FallingBoxRobotDescription("robotWithCS", rootBox, bodyBox, mass, false);
      FallingBoxCollisionMeshDefinitionDataHolder collisionMeshData = new FallingBoxCollisionMeshDefinitionDataHolder(descriptionWithCS);
      descriptionWithCS.addCollisionMeshDefinitionData(collisionMeshData);
      Robot robotWithCS = new RobotFromDescription(descriptionWithCS);
      FloatingJoint floatingJointTwo = (FloatingJoint) robotWithCS.getRootJoints().get(0);
      floatingJointTwo.setPosition(initialPositionTwo);
      floatingJointTwo.setQuaternion(initialOrientation);
      FallingBoxRobotCSBasedEstimator csBasedEstimator = new FallingBoxRobotCSBasedEstimator(robotWithCS, dt);
      robotWithCS.setController(csBasedEstimator);

      List<Robot> allSimulatedRobotList = new ArrayList<Robot>();
      allSimulatedRobotList.add(robotWithGCP);
      allSimulatedRobotList.add(robotWithCS);

      // Env has static collision shape.
      FlatGroundEnvironment environment = new FlatGroundEnvironment();

      // scs.
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);

      SimulationConstructionSet scs = new SimulationConstructionSet(allSimulatedRobotList.toArray(new Robot[0]), parameters);
      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(staticLinkGraphics);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());

      // simulate.
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, 0.01, scs, 1000);
      double coefficientOfRestitution = 0.2;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      scs.initializeCollisionDetector(collisionVisualizer, collisionHandler);
      scs.addEnvironmentCollisionShapes(environment.getTerrainObject3D().getSimpleShapes());
      scs.initializeCollisionHandler(collisionVisualizer, collisionHandler);

      scs.setDT(dt, 1);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      // run.
      scs.setCameraFix(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, -8.0, 1.4);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new FallingBoxSimulation();
   }
}
