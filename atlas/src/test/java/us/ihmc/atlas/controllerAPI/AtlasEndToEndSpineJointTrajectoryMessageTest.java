package us.ihmc.atlas.controllerAPI;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndSpineJointTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasEndToEndSpineJointTrajectoryMessageTest extends EndToEndSpineJointTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test (timeout = 100000)
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test (timeout = 100000)
   public void testSwitchingBetweenControlModes() throws SimulationExceededMaximumTimeException
   {
      super.testSwitchingBetweenControlModes();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test (timeout = 100000)
   public void testDesiredsAreContinuous() throws SimulationExceededMaximumTimeException
   {
      super.testDesiredsAreContinuous();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test (timeout = 100000)
   public void testMultipleWaypoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleWaypoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test (timeout = 100000)
   public void testLongMessage() throws SimulationExceededMaximumTimeException
   {
      super.testLongMessage();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test (timeout = 100000)
   public void testMessageQueuing() throws SimulationExceededMaximumTimeException
   {
      super.testMessageQueuing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test (timeout = 100000)
   public void testMessageWithDifferentTrajectoryLengthsPerJoint() throws SimulationExceededMaximumTimeException
   {
      super.testMessageWithDifferentTrajectoryLengthsPerJoint();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

}
