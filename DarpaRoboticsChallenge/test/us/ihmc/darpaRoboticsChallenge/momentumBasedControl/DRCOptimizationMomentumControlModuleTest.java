package us.ihmc.darpaRoboticsChallenge.momentumBasedControl;

import static org.junit.Assert.assertTrue;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertRootJointWrenchZero;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesInFrictionCones;
import static us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlTestTools.assertWrenchesSumUpToMomentumDot;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;

import javax.xml.bind.JAXBException;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.MatrixFeatures;
import org.junit.Test;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCLocalConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;
import us.ihmc.darpaRoboticsChallenge.initialSetup.SquaredUpDRCRobotInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.geometry.CenterOfMassReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

/**
 * @author twan
 *         Date: 5/7/13
 */
public class DRCOptimizationMomentumControlModuleTest
{
   @Test
   public void testAllJointAccelerationsZero() throws IOException, JAXBException
   {
      Random random = new Random(1252515L);

      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCLocalConfigParameters.robotModelToUse, false);
      JaxbSDFLoader jaxbSDFLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      SDFFullRobotModel fullRobotModel = jaxbSDFLoader.createFullRobotModel(jointMap);
      SDFRobot robot = jaxbSDFLoader.createRobot(jointMap, false);
      ReferenceFrames referenceFrames = new ReferenceFrames(fullRobotModel, jointMap, jointMap.getAnkleHeight());

      RobotInitialSetup<SDFRobot> intialSetup = new SquaredUpDRCRobotInitialSetup();
      initializeRobot(fullRobotModel, robot, referenceFrames, intialSetup);

      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());
      ScrewTestTools.setRandomVelocities(allJoints, random);
      OneDoFJoint[] oneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);

      SideDependentList<ContactablePlaneBody> feet = createFeet(fullRobotModel, referenceFrames);
      YoVariableRegistry registry = new YoVariableRegistry("test");
      double coefficientOfFriction = 1.0;

      LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = createContactStates(feet, registry, coefficientOfFriction);

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      centerOfMassFrame.update();
      OneDoFJoint lidarJoint = fullRobotModel.getOneDoFJointByName(jointMap.getLidarJointName());
      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerFactoryHelper.computeJointsToOptimizeFor(fullRobotModel, lidarJoint);

      double controlDT = 1e-4;
      MomentumOptimizationSettings optimizationSettings = createOptimizationSettings(jointsToOptimizeFor, 0.0, 1e-3, 1e-9, 0.0, 0.0);
      double gravityZ = 9.81;
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      twistCalculator.compute();
      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, controlDT, gravityZ,
            optimizationSettings, twistCalculator, null, contactStates.values(), null, registry);
      momentumControlModule.initialize();

      double mass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));
      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.integrateVelocities(rootJoint, controlDT);
         ScrewTestTools.integrateVelocities(Arrays.asList(oneDoFJoints), controlDT);
         fullRobotModel.updateFrames();
         centerOfMassFrame.update();

         momentumControlModule.reset();

         for (InverseDynamicsJoint inverseDynamicsJoint : jointsToOptimizeFor)
         {
            DenseMatrix64F vdDesired = new DenseMatrix64F(inverseDynamicsJoint.getDegreesOfFreedom(), 1);
            
            
            DesiredJointAccelerationCommand desiredJointAccelerationCommand = new DesiredJointAccelerationCommand(inverseDynamicsJoint, vdDesired);
            momentumControlModule.setDesiredJointAcceleration(desiredJointAccelerationCommand);
         }

         MomentumModuleSolution momentumModuleSolution;
         try
         {
            momentumModuleSolution = momentumControlModule.compute(contactStates, null);
         }
         catch (MomentumControlModuleException momentumControlModuleException)
         {
            throw new RuntimeException();
         }
        
         for (InverseDynamicsJoint inverseDynamicsJoint : jointsToOptimizeFor)
         {
            DenseMatrix64F vdDesired = new DenseMatrix64F(inverseDynamicsJoint.getDegreesOfFreedom(), 1);
            inverseDynamicsJoint.packDesiredAccelerationMatrix(vdDesired, 0);
            assertTrue(MatrixFeatures.isConstantVal(vdDesired, 0.0, 1e-5));
         }

         Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();

         assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumModuleSolution.getCentroidalMomentumRateSolution(),
                                          gravityZ, mass, centerOfMassFrame, 1e-3);
         assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);
      }
   }

   @Test
   public void testStandingInDoubleSupport() throws NoConvergenceException
   {
      Random random = new Random(1252515L);

      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCLocalConfigParameters.robotModelToUse, false);
      JaxbSDFLoader jaxbSDFLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      SDFFullRobotModel fullRobotModel = jaxbSDFLoader.createFullRobotModel(jointMap);
      SDFRobot robot = jaxbSDFLoader.createRobot(jointMap, false);
      ReferenceFrames referenceFrames = new ReferenceFrames(fullRobotModel, jointMap, jointMap.getAnkleHeight());

      RobotInitialSetup<SDFRobot> intialSetup = new SquaredUpDRCRobotInitialSetup();
      initializeRobot(fullRobotModel, robot, referenceFrames, intialSetup);

      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      RigidBody elevator = fullRobotModel.getElevator();

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());

//    ScrewTestTools.setRandomVelocities(allJoints, random);
      OneDoFJoint[] oneDoFJoints = ScrewTools.filterJoints(allJoints, OneDoFJoint.class);

      SideDependentList<ContactablePlaneBody> feet = createFeet(fullRobotModel, referenceFrames);
      YoVariableRegistry registry = new YoVariableRegistry("test");
      double coefficientOfFriction = 1.0;

      LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = createContactStates(feet, registry, coefficientOfFriction);

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      OneDoFJoint lidarJoint = fullRobotModel.getOneDoFJointByName(jointMap.getLidarJointName());
      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerFactoryHelper.computeJointsToOptimizeFor(fullRobotModel, lidarJoint);

      double controlDT = DRCConfigParameters.CONTROL_DT;
      MomentumOptimizationSettings optimizationSettings = createOptimizationSettings(jointsToOptimizeFor, 0.0, 0.0, 1e-5, 0.0, 0.0);
      double gravityZ = 9.81;
      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getSuccessor());
      twistCalculator.compute();
      OptimizationMomentumControlModule momentumControlModule = new OptimizationMomentumControlModule(rootJoint, centerOfMassFrame, controlDT, gravityZ,
            optimizationSettings, twistCalculator, null, contactStates.values(), null, registry);
      momentumControlModule.initialize();

      double mass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));

      for (int i = 0; i < 100; i++)
      {
         ScrewTestTools.integrateVelocities(rootJoint, controlDT);
         ScrewTestTools.integrateVelocities(Arrays.asList(oneDoFJoints), controlDT);
         fullRobotModel.updateFrames();
         centerOfMassFrame.update();

         momentumControlModule.reset();

         Map<GeometricJacobian, TaskspaceConstraintData> taskspaceConstraintDataMap = new HashMap<GeometricJacobian, TaskspaceConstraintData>();

         constrainFeet(elevator, feet, momentumControlModule, taskspaceConstraintDataMap);
         constrainPelvis(random, fullRobotModel, momentumControlModule, taskspaceConstraintDataMap);

         MomentumModuleSolution momentumModuleSolution = momentumControlModule.compute(contactStates, null);
         Map<RigidBody, Wrench> externalWrenchSolution = momentumModuleSolution.getExternalWrenchSolution();
         
         assertWrenchesSumUpToMomentumDot(externalWrenchSolution.values(), momentumModuleSolution.getCentroidalMomentumRateSolution(),
                                          gravityZ, mass, centerOfMassFrame, 1e-3);
         assertWrenchesInFrictionCones(externalWrenchSolution, contactStates, coefficientOfFriction);

         for (GeometricJacobian jacobian : taskspaceConstraintDataMap.keySet())
         {
            assertSpatialAccelerationCorrect(jacobian.getBase(), jacobian.getEndEffector(), taskspaceConstraintDataMap.get(jacobian));
         }

         assertRootJointWrenchZero(externalWrenchSolution, rootJoint, gravityZ, 1e-2);
      }
   }


   private void assertSpatialAccelerationCorrect(RigidBody base, RigidBody endEffector, TaskspaceConstraintData taskspaceConstraintData)
   {
      RigidBody elevator = ScrewTools.getRootBody(base);
      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
                                                                       true, true);
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      SpatialAccelerationVector accelerationBack = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(accelerationBack, base, endEffector);

      DenseMatrix64F selectionMatrix = taskspaceConstraintData.getSelectionMatrix();
      DenseMatrix64F accelerationBackSelection = computeAccelerationSelection(accelerationBack, selectionMatrix);
      DenseMatrix64F accelerationInputSelection = computeAccelerationSelection(taskspaceConstraintData.getSpatialAcceleration(), selectionMatrix);

      EjmlUnitTests.assertEquals(accelerationInputSelection, accelerationBackSelection, 1e-5);
   }

   private DenseMatrix64F computeAccelerationSelection(SpatialAccelerationVector acceleration, DenseMatrix64F selectionMatrix)
   {
      DenseMatrix64F accelerationBackMatrix = new DenseMatrix64F(SpatialAccelerationVector.SIZE, 1);
      acceleration.packMatrix(accelerationBackMatrix, 0);

      DenseMatrix64F accelerationSelection = new DenseMatrix64F(selectionMatrix.getNumRows(), 1);
      CommonOps.mult(selectionMatrix, accelerationBackMatrix, accelerationSelection);

      return accelerationSelection;
   }

   private void constrainPelvis(Random random, SDFFullRobotModel fullRobotModel, OptimizationMomentumControlModule momentumControlModule,
                                Map<GeometricJacobian, TaskspaceConstraintData> taskspaceConstraintDataMap)
   {
      RigidBody pelvis = fullRobotModel.getRootJoint().getSuccessor();
      RigidBody elevator = fullRobotModel.getElevator();
      GeometricJacobian rootJointJacobian = new GeometricJacobian(pelvis, elevator, pelvis.getBodyFixedFrame());
      TaskspaceConstraintData pelvisTaskspaceConstraintData = new TaskspaceConstraintData();
      SpatialAccelerationVector pelvisSpatialAcceleration = new SpatialAccelerationVector(rootJointJacobian.getEndEffectorFrame(),
                                                               rootJointJacobian.getBaseFrame(), rootJointJacobian.getJacobianFrame());
      pelvisSpatialAcceleration.setAngularPart(RandomTools.generateRandomVector(random));

//    pelvisSpatialAcceleration.setAngularPart(new Vector3d());
      DenseMatrix64F pelvisNullspaceMultipliers = new DenseMatrix64F(0, 1);
      DenseMatrix64F orientationSelectionMatrix = new DenseMatrix64F(3, Momentum.SIZE);
      CommonOps.setIdentity(orientationSelectionMatrix);
      pelvisTaskspaceConstraintData.set(pelvisSpatialAcceleration, pelvisNullspaceMultipliers, orientationSelectionMatrix);
      
      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(rootJointJacobian, pelvisTaskspaceConstraintData);
      momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
      taskspaceConstraintDataMap.put(rootJointJacobian, pelvisTaskspaceConstraintData);
   }

   private void constrainFeet(RigidBody elevator, SideDependentList<ContactablePlaneBody> feet, OptimizationMomentumControlModule momentumControlModule,
                              Map<GeometricJacobian, TaskspaceConstraintData> taskspaceConstraintDataMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = feet.get(robotSide).getRigidBody();
         GeometricJacobian jacobian = new GeometricJacobian(elevator, foot, foot.getBodyFixedFrame());
         TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
         SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(foot.getBodyFixedFrame(), elevator.getBodyFixedFrame(),
                                                            foot.getBodyFixedFrame());
         taskspaceConstraintData.set(spatialAcceleration);
         
         DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskspaceConstraintData);
         momentumControlModule.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
         taskspaceConstraintDataMap.put(jacobian, taskspaceConstraintData);
      }
   }

   private void initializeRobot(SDFFullRobotModel fullRobotModel, SDFRobot robot, ReferenceFrames referenceFrames, RobotInitialSetup<SDFRobot> intialSetup)
   {
      intialSetup.initializeRobot(robot);
      SDFPerfectSimulatedSensorReader sensorReader = new SDFPerfectSimulatedSensorReader(robot, fullRobotModel, referenceFrames);
      sensorReader.read();
   }

   private LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> createContactStates(SideDependentList<ContactablePlaneBody> feet,
           YoVariableRegistry registry, double coefficientOfFriction)
   {
      LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();
      for (ContactablePlaneBody contactablePlaneBody : feet)
      {
         String contactStateName = contactablePlaneBody.getName() + "ContactState";
         YoPlaneContactState contactState = new YoPlaneContactState(contactStateName, contactablePlaneBody.getRigidBody(),
                                               contactablePlaneBody.getPlaneFrame(), contactablePlaneBody.getContactPoints2d(), coefficientOfFriction, registry);
         contactStates.put(contactablePlaneBody, contactState);
      }

      return contactStates;
   }

   private SideDependentList<ContactablePlaneBody> createFeet(SDFFullRobotModel fullRobotModel, ReferenceFrames referenceFrames)
   {
      double footForward = DRCRobotParameters.DRC_ROBOT_FOOT_FORWARD;
      double footBack = DRCRobotParameters.DRC_ROBOT_FOOT_BACK;
      double footWidth = DRCRobotParameters.DRC_ROBOT_FOOT_WIDTH;

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      return bipedFeet;
   }

   private static MomentumOptimizationSettings createOptimizationSettings(InverseDynamicsJoint[] jointsToOptimizeFor, double momentumWeight, double lambda, double wRho, double rhoMin, double wPhi)
   {
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(jointsToOptimizeFor, new YoVariableRegistry("test1"));
      momentumOptimizationSettings.setMomentumWeight(momentumWeight, momentumWeight, momentumWeight, momentumWeight);
      momentumOptimizationSettings.setDampedLeastSquaresFactor(lambda);
      momentumOptimizationSettings.setRhoPlaneContactRegularization(wRho);
      momentumOptimizationSettings.setPhiCylinderContactRegularization(wPhi);
      momentumOptimizationSettings.setRhoMin(rhoMin);

      return momentumOptimizationSettings;
   }
}
