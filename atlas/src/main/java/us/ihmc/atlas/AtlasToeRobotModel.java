package us.ihmc.atlas;

import java.util.EnumMap;

import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasContinuousCMPPlannerParameters;
import us.ihmc.atlas.parameters.AtlasLegConfigurationParameters;
import us.ihmc.atlas.parameters.AtlasMomentumOptimizationSettings;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSwingTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasToeOffParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasToeRobotModel extends AtlasRobotModel {

	public AtlasToeRobotModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless){
		super(atlasVersion, target, headless);

	}

	@Override
	public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight,
			double initialYaw) {
		return new AtlasSimSetup(groundHeight, initialYaw);
	}

	@Override
	public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
	{
		boolean enableTorqueVelocityLimits = false;
		HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot = new HumanoidFloatingRootJointRobot(super.getRobotDescription(), super.getJointMap(), enableJointDamping,
				enableTorqueVelocityLimits);

		// adding ground contact points for the ankle roll joint.
		for (RobotSide robotSide : RobotSide.values)
			humanoidFloatingRootJointRobot.addFootGroundContactPoints(robotSide,humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoint(super.getJointMap().getLegJointName(robotSide, LegJointName.ANKLE_ROLL)));

		return humanoidFloatingRootJointRobot;
	}
	
	@Override
	public WalkingControllerParameters getWalkingControllerParameters()
	{
		// momentum, swing, toe-off and other parameters.
		return new TestWalkingControllerParameters(getJointMap(), getContactPointParameters());
	}

	@Override
	public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
	{
		// changes COP offset for walking trajectories. Tuning parameters.
		return new TestICPPlannerParameters(getPhysicalProperties());
	}


	private class AtlasSimSetup extends AtlasSimInitialSetup {

		public AtlasSimSetup(double groundHeight, double initialYaw) {
			super(groundHeight, initialYaw);
		}

		private boolean robotInitialized = false;

		@Override
		public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap) {

			if(!robotInitialized)
			{
				robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(RobotSide.LEFT, LegJointName.TOE_PITCH)).setQ(0.0); 	//l_leg_toe
				robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.TOE_PITCH)).setQ(0.0); 	//r_leg_toe
				robotInitialized = true;
			}

			super.initializeRobot(robot, jointMap);

		}
	}
	
	private class TestWalkingControllerParameters extends AtlasWalkingControllerParameters
	   {
	      private final AtlasJointMap jointMap;
	      private final AtlasContactPointParameters contactPointParameters;

	      public TestWalkingControllerParameters(AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
	      {
	         super(RobotTarget.SCS, jointMap, contactPointParameters);

	         this.jointMap = jointMap;
	         this.contactPointParameters = contactPointParameters;
	      }

	      @Override
	      public boolean controlHeightWithMomentum()
	      {
	         // changes selection matrix. Now height governed by leg configurations.
	    	  return false;
	      }

	      @Override
	      public boolean useOptimizationBasedICPController()
	      {
	         // uses icp optimization controller.
	    	  return true;
	      }

	      @Override
	      public boolean editStepTimingForReachability()
	      {
	         // makes edit in timings to make sure step is reachable. default is false.
	    	  return true;
	      }

	      @Override
	      public boolean applySecondaryJointScaleDuringSwing()
	      {
	         // didn't understand quite well
	    	  return true;
	      }

	      @Override
	      public LeapOfFaithParameters getLeapOfFaithParameters()
	      {
	         // parameters showing if weight should change at end of swing to get the leg down. rotate pelvis, etc.
	    	  return new TestLeapOfFaithParameters();
	      }

	      @Override
	      public LegConfigurationParameters getLegConfigurationParameters()
	      {
	         // attempts to straighten leg and sets objective weights
	    	  return new TestLegConfigurationParameters();
	      }

	      @Override
	      public MomentumOptimizationSettings getMomentumOptimizationSettings()
	      {
	         // increase joint acceleration weight.slows the system down.
	    	  return new TestMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());
	      }

	      @Override
	      public SwingTrajectoryParameters getSwingTrajectoryParameters()
	      {
	         // do not avoid singularities in stance and swing. do heel and toe touchdown if possible.
	    	  return new TestSwingTrajectoryParameters();
	      }

	      @Override
	      public TestToeOffParameters getToeOffParameters()
	      {
	    	  // toe off criterias defined
	         return new TestToeOffParameters(jointMap);
	      }

	   }

	   private class TestToeOffParameters extends AtlasToeOffParameters
	   {
	      public TestToeOffParameters(AtlasJointMap jointMap)
	      {
	         super(jointMap);
	      }

	      @Override
	      public boolean checkCoPLocationToTriggerToeOff()
	      {
	         return true;
	      }

	      @Override
	      public double getCoPProximityForToeOff()
	      {
	         return 0.05;
	      }

	      @Override
	      public double getICPPercentOfStanceForDSToeOff()
	      {
	         return 0.20;
	      }

	      @Override
	      public double getICPPercentOfStanceForSSToeOff()
	      {
	         return 0.10;
	      }

	      @Override
	      public boolean checkECMPLocationToTriggerToeOff()
	      {
	         return true;
	      }

	      @Override
	      public double getECMPProximityForToeOff()
	      {
	         return 0.01;
	      }

	      @Override
	      public boolean doToeOffIfPossibleInSingleSupport()
	      {
	         return true;
	      }

	      @Override
	      public double getAnkleLowerLimitToTriggerToeOff()
	      {
	         return -0.75;
	      }
	      
	      @Override
	      public boolean useToeOffLineContactInTransfer()
	      {
	         return false; // default : false
	      }
	      
	      @Override
	      public boolean useToeOffLineContactInSwing()
	      {
	         return true; // default : true
	      }

	   }

	   private class TestSwingTrajectoryParameters extends AtlasSwingTrajectoryParameters
	   {
	      public TestSwingTrajectoryParameters()
	      {
	         super(RobotTarget.SCS, 1.0);
	      }


	      @Override
	      public boolean useSingularityAvoidanceInSwing()
	      {
	         return false;
	      }

	      @Override
	      public boolean useSingularityAvoidanceInSupport()
	      {
	         return false;
	      }

	      @Override
	      public boolean doHeelTouchdownIfPossible()
	      {
	         return true;
	      }


	      @Override
	      public boolean doToeTouchdownIfPossible()
	      {
	         return true;
	      }

	      @Override
	      public boolean addOrientationMidpointForObstacleClearance()
	      {
	         return true;
	      }
	   }

	   private class TestLeapOfFaithParameters extends LeapOfFaithParameters
	   {
	      @Override
	      public boolean scaleFootWeight()
	      {
	         return true;
	      }

	      @Override
	      public boolean usePelvisRotation()
	      {
	         return true;
	      }

	      @Override
	      public boolean relaxPelvisControl()
	      {
	         return true;
	      }

	      @Override
	      public double getRelaxationRate()
	      {
	         return 2.0;
	      }

	      @Override
	      public double getMinimumPelvisWeight()
	      {
	         return 0.5;
	      }
	   }

	   private class TestLegConfigurationParameters extends AtlasLegConfigurationParameters
	   {
	      public TestLegConfigurationParameters()
	      {
	         super(false);
	      }

	      @Override
	      public boolean attemptToStraightenLegs()
	      {
	         return true;
	      }

	      @Override
	      public double getLegPrivilegedLowWeight()
	      {
	         return 5.0;
	      }

	      @Override
	      public double getLegPrivilegedMediumWeight()
	      {
	         return 75.0;
	      }

	      @Override
	      public double getLegPrivilegedHighWeight()
	      {
	         return 150.0;
	      }
	      
	      @Override
	      public boolean toeOffWhenCollapsed()
	      {
	    	 return true; 
	      }
	   }

	   private class TestMomentumOptimizationSettings extends AtlasMomentumOptimizationSettings
	   {
	      public TestMomentumOptimizationSettings(AtlasJointMap jointMap, int numberOfContactableBodies)
	      {
	         super(jointMap, numberOfContactableBodies);
	      }

	      @Override
	      public double getJointAccelerationWeight()
	      {
	         // increased by 10 times than usual. Maybe because we want the system to run slowly.
	    	  return 0.05;
	      }
	   }

	   private class TestICPPlannerParameters extends AtlasContinuousCMPPlannerParameters
	   {
	      public TestICPPlannerParameters(AtlasPhysicalProperties physicalProperties)
	      {
	         super(physicalProperties);
	      }

	      @Override
	      public double getExitCoPForwardSafetyMarginOnToes()
	      {
	         return 0.015;
	      }

	      @Override
	      public boolean putExitCoPOnToes()
	      {
	         return true;
	      }

	      /** {@inheritDoc} */
	      @Override
	      public EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame()
	      {
	         Vector2D entryOffset = new Vector2D(0.0, -0.005);
	         Vector2D exitOffset = new Vector2D(0.0, 0.015);

	         EnumMap<CoPPointName, Vector2D> copOffsets = new EnumMap<>(CoPPointName.class);
	         copOffsets.put(entryCoPName, entryOffset);
	         copOffsets.put(exitCoPName, exitOffset);

	         return copOffsets;
	      }
	   }

}

