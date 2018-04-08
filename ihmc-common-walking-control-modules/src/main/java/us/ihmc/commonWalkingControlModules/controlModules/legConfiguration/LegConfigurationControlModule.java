package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachineTools;

import java.util.ArrayList;
import java.util.List;

public class LegConfigurationControlModule
{
	public enum LegConfigurationType
	{
		STRAIGHTEN, STRAIGHT, COLLAPSE, BENT
	}

	public enum LegControlWeight
	{
		HIGH, MEDIUM, LOW
	}

	private static final double minimumDampingScale = 0.2;
	private static final boolean scaleDamping = true;

	private static final boolean ONLY_MOVE_PRIV_POS_IF_NOT_BENDING = true;

	private final YoVariableRegistry registry;

	private final PrivilegedAccelerationCommand privilegedAccelerationCommand = new PrivilegedAccelerationCommand();

	private final YoEnum<LegConfigurationType> requestedState;
	private final YoEnum<LegControlWeight> legControlWeight;

	private final GenericStateMachine<LegConfigurationType, FinishableState<LegConfigurationType>> stateMachine;

	private final YoDouble highPrivilegedWeight;
	private final YoDouble mediumPrivilegedWeight;
	private final YoDouble lowPrivilegedWeight;

	private final YoDouble straightJointSpacePositionGain;
	private final YoDouble straightJointSpaceVelocityGain;
	private final YoDouble straightActuatorSpacePositionGain;
	private final YoDouble straightActuatorSpaceVelocityGain;
	private final YoDouble straightMaxPositionBlendingFactor;
	private final YoDouble straightMaxVelocityBlendingFactor;
	private final YoBoolean straightUseActuatorSpacePositionControl;
	private final YoBoolean straightUseActuatorSpaceVelocityControl;

	private final YoDouble bentJointSpacePositionGain;
	private final YoDouble bentJointSpaceVelocityGain;
	private final YoDouble bentActuatorSpaceVelocityGain;
	private final YoDouble bentActuatorSpacePositionGain;
	private final YoDouble bentMaxPositionBlendingFactor;
	private final YoDouble bentMaxVelocityBlendingFactor;
	private final YoBoolean bentUseActuatorSpacePositionControl;
	private final YoBoolean bentUseActuatorSpaceVelocityControl;

	private final YoDouble kneePitchPrivilegedConfiguration;
	private final YoDouble kneePitchPrivilegedError;
	private final YoDouble toePitchPrivilegedConfiguration;
	private final YoDouble toePitchPrivilegedError;
	private final YoDouble anklePitchPrivilegedConfiguration;
	private final YoDouble anklePitchPrivilegedError;

	private final YoDouble kneePrivilegedPAction;
	private final YoDouble kneePrivilegedDAction;
	private final YoDouble privilegedMaxAcceleration;

	private final YoDouble toePrivilegedPAction;
	private final YoDouble toePrivilegedDAction;

	private final YoDouble effectiveKneeStiffness;
	private final YoDouble effectiveKneeDamping;

	private final YoBoolean useFullyExtendedLeg;
	private final YoBoolean useBracingAngle;
	private final YoDouble desiredAngle;
	private final YoDouble desiredAngleWhenStraight;
	private final YoDouble desiredAngleWhenExtended;
	private final YoDouble desiredAngleWhenBracing;

	private final YoDouble desiredFractionOfMidRangeForCollapsed;
	private final YoBoolean toeOffWhenCollapsed; 

	private final YoDouble straighteningSpeed;
	private final YoDouble collapsingDuration;
	private final YoDouble collapsingDurationFractionOfStep;

	private final YoDouble desiredVirtualActuatorLength;
	private final YoDouble currentVirtualActuatorLength;
	private final YoDouble currentVirtualActuatorVelocity;

	private final OneDoFJoint kneePitchJoint;
	private final OneDoFJoint toePitchJoint;
	private final OneDoFJoint anklePitchJoint;

	private final YoDouble positionBlendingFactor;
	private final YoDouble velocityBlendingFactor;
	private final YoDouble dampingActionScaleFactor;

	private static final int hipPitchJointIndex = 0;
	private static final int kneePitchJointIndex = 1;
	private static final int anklePitchJointIndex = 2;
	private static final int toePitchJointIndex = 3;

	private double jointSpaceConfigurationGain;
	private double jointSpaceVelocityGain;
	private double actuatorSpaceConfigurationGain;
	private double actuatorSpaceVelocityGain;
	private double maxPositionBlendingFactor;
	private double maxVelocityBlendingFactor;

	private boolean useActuatorSpacePositionControl;
	private boolean useActuatorSpaceVelocityControl;

	private boolean blendPositionError;
	private boolean blendVelocityError;

	private final double kneeRangeOfMotion;
	private final double kneeSquareRangeOfMotion;
	private final double kneeMidRangeOfMotion;

	private final double thighLength;
	private final double shinLength;

	private final LegConfigurationGains straightLegGains;
	private final LegConfigurationGains bentLegGains;
	
	private YoMinimumJerkTrajectory trajectoryToePitch;
	private YoMinimumJerkTrajectory trajectoryAnklePitch;
	
	private final YoDouble anklePitchPrivilegedAcceleration;
	private final YoDouble toePitchPrivilegedAcceleration;



	public LegConfigurationControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, LegConfigurationParameters legConfigurationParameters,
			YoVariableRegistry parentRegistry)
	{
		String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
		String namePrefix = sidePrefix + "Leg";
		registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());

		kneePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);
		toePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.TOE_PITCH);
		anklePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);

		double kneeLimitUpper = kneePitchJoint.getJointLimitUpper();
		if (Double.isNaN(kneeLimitUpper) || Double.isInfinite(kneeLimitUpper))
			kneeLimitUpper = Math.PI;
		double kneeLimitLower = kneePitchJoint.getJointLimitLower();
		if (Double.isNaN(kneeLimitLower) || Double.isInfinite(kneeLimitLower))
			kneeLimitLower = -Math.PI;
		kneeSquareRangeOfMotion = MathTools.square(kneeLimitUpper - kneeLimitLower);
		kneeRangeOfMotion = kneeLimitUpper - kneeLimitLower;
		kneeMidRangeOfMotion = 0.5 * (kneeLimitUpper + kneeLimitLower);

		toeOffWhenCollapsed = new YoBoolean("toeOffWhenCollapsed", registry);
		toeOffWhenCollapsed.set(legConfigurationParameters.toeOffWhenCollapsed());

		OneDoFJoint hipPitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
		privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
		privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
		privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);

		if(toeOffWhenCollapsed.getBooleanValue())
		{
			privilegedAccelerationCommand.addJoint(toePitchJoint, Double.NaN);
			toePitchPrivilegedError = new YoDouble(sidePrefix + "ToePitchPrivilegedError", registry);
			toePitchPrivilegedConfiguration = new YoDouble(sidePrefix + "ToePitchPrivilegedConfiguration", registry);
			anklePitchPrivilegedError = new YoDouble(sidePrefix + "AnklePitchPrivilegedError", registry);
			anklePitchPrivilegedConfiguration = new YoDouble(sidePrefix + "AnklePitchPrivilegedConfiguration", registry);
			toePrivilegedPAction = new YoDouble(sidePrefix + "ToePrivilegedPAction", registry);
			toePrivilegedDAction = new YoDouble(sidePrefix + "ToePrivilegedDAction", registry);
			anklePitchPrivilegedAcceleration =  new YoDouble(sidePrefix + "AnklePitchPrivilegedAcceleration", registry);
			toePitchPrivilegedAcceleration =  new YoDouble(sidePrefix + "ToePitchPrivilegedAcceleration", registry);
			trajectoryToePitch = new YoMinimumJerkTrajectory("TrajectoryToePitch", registry);
			trajectoryAnklePitch = new YoMinimumJerkTrajectory("TrajectoryAnklePitch", registry);
		}
		else 
		{
			toePitchPrivilegedError = null;
			toePrivilegedPAction = null;
			toePrivilegedDAction = null;
			toePitchPrivilegedConfiguration = null;
			anklePitchPrivilegedConfiguration = null;
			anklePitchPrivilegedError = null;
			trajectoryToePitch = null;
			trajectoryAnklePitch = null;
			anklePitchPrivilegedAcceleration = null;
			toePitchPrivilegedAcceleration = null;
		} 

		highPrivilegedWeight = new YoDouble(sidePrefix + "HighPrivilegedWeight", registry);
		mediumPrivilegedWeight = new YoDouble(sidePrefix + "MediumPrivilegedWeight", registry);
		lowPrivilegedWeight = new YoDouble(sidePrefix + "LowPrivilegedWeight", registry);

		straightJointSpacePositionGain = new YoDouble(sidePrefix + "StraightLegJointSpaceKp", registry);
		straightJointSpaceVelocityGain = new YoDouble(sidePrefix + "StraightLegJointSpaceKv", registry);
		straightActuatorSpacePositionGain = new YoDouble(sidePrefix + "StraightLegActuatorSpaceKp", registry);
		straightActuatorSpaceVelocityGain = new YoDouble(sidePrefix + "StraightLegActuatorSpaceKv", registry);
		straightMaxPositionBlendingFactor = new YoDouble(sidePrefix + "StraightMaxPositionBlendingFactor", registry);
		straightMaxVelocityBlendingFactor = new YoDouble(sidePrefix + "StraightMaxVelocityBlendingFactor", registry);
		straightUseActuatorSpacePositionControl = new YoBoolean(sidePrefix + "StraightUseActuatorSpacePositionControl", registry);
		straightUseActuatorSpaceVelocityControl = new YoBoolean(sidePrefix + "StraightUseActuatorSpaceVelocityControl", registry);

		bentJointSpacePositionGain = new YoDouble(sidePrefix + "BentLegJointSpaceKp", registry);
		bentJointSpaceVelocityGain = new YoDouble(sidePrefix + "BentLegJointSpaceKv", registry);
		bentActuatorSpacePositionGain = new YoDouble(sidePrefix + "BentLegActuatorSpaceKp", registry);
		bentActuatorSpaceVelocityGain = new YoDouble(sidePrefix + "BentLegActuatorSpaceKv", registry);
		bentMaxPositionBlendingFactor = new YoDouble(sidePrefix + "BentMaxPositionBlendingFactor", registry);
		bentMaxVelocityBlendingFactor = new YoDouble(sidePrefix + "BentMaxVelocityBlendingFactor", registry);
		bentUseActuatorSpacePositionControl = new YoBoolean(sidePrefix + "BentUseActuatorSpacePositionControl", registry);
		bentUseActuatorSpaceVelocityControl = new YoBoolean(sidePrefix + "BentUseActuatorSpaceVelocityControl", registry);

		kneePitchPrivilegedConfiguration = new YoDouble(sidePrefix + "KneePitchPrivilegedConfiguration", registry);
		privilegedMaxAcceleration = new YoDouble(sidePrefix + "LegPrivilegedMaxAcceleration", registry);

		kneePitchPrivilegedError = new YoDouble(sidePrefix + "KneePitchPrivilegedError", registry);
		kneePrivilegedPAction = new YoDouble(sidePrefix + "KneePrivilegedPAction", registry);
		kneePrivilegedDAction = new YoDouble(sidePrefix + "KneePrivilegedDAction", registry);

		effectiveKneeStiffness = new YoDouble(sidePrefix + "EffectiveKneeStiffness", registry);
		effectiveKneeDamping = new YoDouble(sidePrefix + "EffectiveKneeDamping", registry);

		highPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedHighWeight());
		mediumPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedMediumWeight());
		lowPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedLowWeight());

		straightLegGains = legConfigurationParameters.getStraightLegGains();
		bentLegGains = legConfigurationParameters.getBentLegGains();

		straightJointSpacePositionGain.set(straightLegGains.getJointSpaceKp());
		straightJointSpaceVelocityGain.set(straightLegGains.getJointSpaceKd());
		straightActuatorSpacePositionGain.set(straightLegGains.getActuatorSpaceKp());
		straightActuatorSpaceVelocityGain.set(straightLegGains.getActuatorSpaceKd());
		straightMaxPositionBlendingFactor.set(straightLegGains.getMaxPositionBlendingFactor());
		straightMaxVelocityBlendingFactor.set(straightLegGains.getMaxVelocityBlendingFactor());
		straightUseActuatorSpacePositionControl.set(straightLegGains.getUseActuatorSpacePositionControl());
		straightUseActuatorSpaceVelocityControl.set(straightLegGains.getUseActuatorSpaceVelocityControl());

		bentJointSpacePositionGain.set(bentLegGains.getJointSpaceKp());
		bentJointSpaceVelocityGain.set(bentLegGains.getJointSpaceKd());
		bentActuatorSpacePositionGain.set(bentLegGains.getActuatorSpaceKp());
		bentActuatorSpaceVelocityGain.set(bentLegGains.getActuatorSpaceKd());
		bentMaxPositionBlendingFactor.set(bentLegGains.getMaxPositionBlendingFactor());
		bentMaxVelocityBlendingFactor.set(bentLegGains.getMaxVelocityBlendingFactor());
		bentUseActuatorSpacePositionControl.set(bentLegGains.getUseActuatorSpacePositionControl());
		bentUseActuatorSpaceVelocityControl.set(bentLegGains.getUseActuatorSpaceVelocityControl());

		privilegedMaxAcceleration.set(legConfigurationParameters.getPrivilegedMaxAcceleration());

		positionBlendingFactor = new YoDouble(namePrefix + "PositionBlendingFactor", registry);
		velocityBlendingFactor = new YoDouble(namePrefix + "VelocityBlendingFactor", registry);
		dampingActionScaleFactor = new YoDouble(namePrefix + "DampingActionScaleFactor", registry);

		useFullyExtendedLeg = new YoBoolean(namePrefix + "UseFullyExtendedLeg", registry);
		useBracingAngle = new YoBoolean(namePrefix + "UseBracingLeg", registry);

		desiredAngle = new YoDouble(namePrefix + "DesiredAngle", registry);

		desiredAngleWhenStraight = new YoDouble(namePrefix + "DesiredAngleWhenStraight", registry);
		desiredAngleWhenExtended = new YoDouble(namePrefix + "DesiredAngleWhenExtended", registry);
		desiredAngleWhenBracing = new YoDouble(namePrefix + "DesiredAngleWhenBracing", registry);
		desiredAngleWhenStraight.set(legConfigurationParameters.getKneeAngleWhenStraight());
		desiredAngleWhenExtended.set(legConfigurationParameters.getKneeAngleWhenExtended());
		desiredAngleWhenBracing.set(legConfigurationParameters.getKneeAngleWhenBracing());

		desiredFractionOfMidRangeForCollapsed = new YoDouble(namePrefix + "DesiredFractionOfMidRangeForCollapsed", registry);
		desiredFractionOfMidRangeForCollapsed.set(legConfigurationParameters.getDesiredFractionOfMidrangeForCollapsedAngle());

		straighteningSpeed = new YoDouble(namePrefix + "SupportKneeStraighteningSpeed", registry);
		straighteningSpeed.set(legConfigurationParameters.getSpeedForSupportKneeStraightening());

		collapsingDuration = new YoDouble(namePrefix + "SupportKneeCollapsingDuration", registry);
		collapsingDurationFractionOfStep = new YoDouble(namePrefix + "SupportKneeCollapsingDurationFractionOfStep", registry);
		collapsingDurationFractionOfStep.set(legConfigurationParameters.getSupportKneeCollapsingDurationFractionOfStep());

		desiredVirtualActuatorLength = new YoDouble(namePrefix + "DesiredVirtualActuatorLength", registry);
		currentVirtualActuatorLength = new YoDouble(namePrefix + "CurrentVirtualActuatorLength", registry);
		currentVirtualActuatorVelocity = new YoDouble(namePrefix + "CurrentVirtualActuatorVelocity", registry);



		// set up states and state machine
		YoDouble time = controllerToolbox.getYoTime();
		stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", LegConfigurationType.class, time, registry);
		requestedState = YoEnum.create(namePrefix + "RequestedState", "", LegConfigurationType.class, registry, true);
		requestedState.set(null);
		legControlWeight = YoEnum.create(namePrefix + "LegControlWeight", "", LegControlWeight.class, registry, false);

		// compute leg segment lengths
		FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
		ReferenceFrame hipPitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.HIP_PITCH).getFrameAfterJoint();
		FramePoint3D hipPoint = new FramePoint3D(hipPitchFrame);
		FramePoint3D kneePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameBeforeJoint());
		kneePoint.changeFrame(hipPitchFrame);

		thighLength = hipPoint.distance(kneePoint);

		ReferenceFrame kneePitchFrame = fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getFrameAfterJoint();
		kneePoint.setToZero(kneePitchFrame);
		FramePoint3D anklePoint = new FramePoint3D(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH).getFrameBeforeJoint());
		anklePoint.changeFrame(kneePitchFrame);

		shinLength = kneePoint.distance(anklePoint);

		setupStateMachine();

		if (legConfigurationParameters.attemptToStraightenLegs())
			stateMachine.setCurrentState(LegConfigurationType.STRAIGHT);
		else
			stateMachine.setCurrentState(LegConfigurationType.BENT);

		parentRegistry.addChild(registry);
	}

	private void setupStateMachine()
	{
		List<FinishableState<LegConfigurationType>> states = new ArrayList<>();

		FinishableState<LegConfigurationType> straighteningToStraightState = new StraighteningKneeControlState(straighteningSpeed);
		FinishableState<LegConfigurationType> straightState = new StraightKneeControlState();
		FinishableState<LegConfigurationType> bentState = new BentKneeControlState();
		FinishableState<LegConfigurationType> collapseState = new CollapseKneeControlState();
		states.add(straighteningToStraightState);
		states.add(straightState);
		states.add(bentState);
		states.add(collapseState);

		straighteningToStraightState.setDefaultNextState(LegConfigurationType.STRAIGHT);

		for (FinishableState<LegConfigurationType> fromState : states)
		{
			for (FinishableState<LegConfigurationType> toState : states)
			{
				StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
			}
		}

		for (FinishableState<LegConfigurationType> state : states)
		{
			stateMachine.addState(state);
		}
	}

	public void initialize()
	{
	}

	public void doControl()
	{
		if (useBracingAngle.getBooleanValue())
			desiredAngle.set(desiredAngleWhenBracing.getDoubleValue());
		else if (useFullyExtendedLeg.getBooleanValue())
			desiredAngle.set(desiredAngleWhenExtended.getDoubleValue());
		else
			desiredAngle.set(desiredAngleWhenStraight.getDoubleValue());

		stateMachine.checkTransitionConditions();
		stateMachine.getCurrentState().doAction();

		double kneePitchPrivilegedConfigurationWeight;
		if (legControlWeight.getEnumValue() == LegControlWeight.LOW)
			kneePitchPrivilegedConfigurationWeight = lowPrivilegedWeight.getDoubleValue();
		else if (legControlWeight.getEnumValue() == LegControlWeight.MEDIUM)
			kneePitchPrivilegedConfigurationWeight = mediumPrivilegedWeight.getDoubleValue();
		else
			kneePitchPrivilegedConfigurationWeight = highPrivilegedWeight.getDoubleValue();


		double privilegedKneeAcceleration = computeKneeAcceleration();
		double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
		double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;



		privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
		privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
		privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

		privilegedAccelerationCommand.setWeight(hipPitchJointIndex, kneePitchPrivilegedConfigurationWeight);
		privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneePitchPrivilegedConfigurationWeight);
		privilegedAccelerationCommand.setWeight(anklePitchJointIndex, kneePitchPrivilegedConfigurationWeight);

		if(toeOffWhenCollapsed.getBooleanValue())
		{
			
			double privilegedtoePitchAcceleration = computeToeAcceleration(0.0);
			
			if(stateMachine.getCurrentState().getStateEnum() == LegConfigurationType.STRAIGHTEN)
			{	  
				privilegedtoePitchAcceleration = computeToeAcceleration(0.0); 
				privilegedAccelerationCommand.setWeight(toePitchJointIndex, highPrivilegedWeight.getDoubleValue()+300.0);
			}
			else if(stateMachine.getCurrentState().getStateEnum() == LegConfigurationType.STRAIGHT)
			{	  
				privilegedtoePitchAcceleration = computeToeAcceleration(0.0); 
				privilegedAccelerationCommand.setWeight(toePitchJointIndex, highPrivilegedWeight.getDoubleValue());
			}
			else if(stateMachine.getCurrentState().getStateEnum() == LegConfigurationType.COLLAPSE)
			{	  
				privilegedtoePitchAcceleration = computeToeAcceleration(0.0); 
				privilegedAccelerationCommand.setWeight(toePitchJointIndex, mediumPrivilegedWeight.getDoubleValue());
			}
			else if(stateMachine.getCurrentState().getStateEnum() == LegConfigurationType.BENT)
			{	  
				privilegedtoePitchAcceleration = computeToeAcceleration(0.0);
				privilegedAccelerationCommand.setWeight(toePitchJointIndex, highPrivilegedWeight.getDoubleValue());
			}

			privilegedAccelerationCommand.setOneDoFJoint(toePitchJointIndex, toePitchPrivilegedAcceleration.getDoubleValue());
			

		}
	}

	private double computeToeAcceleration(double desiredPosition)
	{
		toePitchPrivilegedConfiguration.set(desiredPosition);
		double currentPosition = toePitchJoint.getQ();

		double error = desiredPosition - currentPosition;
		toePitchPrivilegedError.set(error);

		double toeRangeOfMotion = Math.PI/2;
		double toeRangeOfMotionSquared = Math.pow(toeRangeOfMotion,2);

		double jointSpaceKp =  100.0*jointSpaceConfigurationGain / toeRangeOfMotionSquared;

		double jointSpacePAction = jointSpaceKp * error;

		double jointSpaceDAction = 10.0*jointSpaceVelocityGain * -toePitchJoint.getQd();

		toePrivilegedPAction.set(jointSpacePAction);
		toePrivilegedDAction.set(jointSpaceDAction);

		toePitchPrivilegedAcceleration.set(MathTools.clamp(toePrivilegedPAction.getDoubleValue() + toePrivilegedDAction.getDoubleValue(), privilegedMaxAcceleration.getDoubleValue()));
		
		return toePitchPrivilegedAcceleration.getDoubleValue();

	}
	
	private double computeAnklePitchAcceleration(double desiredPosition)
	{
		anklePitchPrivilegedConfiguration.set(desiredPosition);
		double currentPosition = anklePitchJoint.getQ();

		double error = desiredPosition - currentPosition;
		anklePitchPrivilegedError.set(error);

		double ankleRangeOfMotion = Math.PI/2;
		double ankleRangeOfMotionSquared = Math.pow(ankleRangeOfMotion,2);

		double jointSpaceKp = 2.0 * jointSpaceConfigurationGain / ankleRangeOfMotionSquared;

		double jointSpacePAction = jointSpaceKp * error;

		double jointSpaceDAction = jointSpaceVelocityGain * -anklePitchJoint.getQd();

		anklePitchPrivilegedAcceleration.set(MathTools.clamp(jointSpacePAction + jointSpaceDAction, privilegedMaxAcceleration.getDoubleValue()));
		
		return anklePitchPrivilegedAcceleration.getDoubleValue();

	}
	
	public void setStepDuration(double stepDuration)
	{
		collapsingDuration.set(collapsingDurationFractionOfStep.getDoubleValue() * stepDuration);
	}

	public void setFullyExtendLeg(boolean fullyExtendLeg)
	{
		useFullyExtendedLeg.set(fullyExtendLeg);
	}

	public void prepareForLegBracing()
	{
		useBracingAngle.set(true);
	}

	public void doNotBrace()
	{
		useBracingAngle.set(false);
	}

	public void setLegControlWeight(LegControlWeight legControlWeight)
	{
		this.legControlWeight.set(legControlWeight);
	}

	private double computeKneeAcceleration()
	{
		double currentPosition = kneePitchJoint.getQ();

		double desiredVirtualLength = computeVirtualActuatorLength(kneePitchPrivilegedConfiguration.getDoubleValue());
		double currentVirtualLength = computeVirtualActuatorLength(currentPosition);

		desiredVirtualActuatorLength.set(desiredVirtualLength);
		currentVirtualActuatorLength.set(currentVirtualLength);

		double error = kneePitchPrivilegedConfiguration.getDoubleValue() - currentPosition;
		double virtualError = desiredVirtualLength - currentVirtualLength;
		kneePitchPrivilegedError.set(error);

		double currentVirtualVelocity = computeVirtualActuatorVelocity(currentPosition, kneePitchJoint.getQd());
		currentVirtualActuatorVelocity.set(currentVirtualVelocity);

		double percentDistanceToMidRange = MathTools.clamp(Math.abs(currentPosition - kneeMidRangeOfMotion) / (0.5 * kneeRangeOfMotion), 0.0, 1.0);
		double positionBlendingFactor = Math.min(maxPositionBlendingFactor * percentDistanceToMidRange, 1.0);
		double velocityBlendingFactor = Math.min(maxVelocityBlendingFactor * percentDistanceToMidRange, 1.0);
		this.positionBlendingFactor.set(positionBlendingFactor);
		this.velocityBlendingFactor.set(velocityBlendingFactor);

		double jointSpaceKp = 2.0 * jointSpaceConfigurationGain / kneeSquareRangeOfMotion;

		double jointSpacePAction = jointSpaceKp * error;
		double actuatorSpacePAction = -actuatorSpaceConfigurationGain * virtualError;

		// modify gains based on error. If there's a big error, don't damp velocities
		double percentError = Math.abs(error) / (0.5 * kneeRangeOfMotion);
		double dampingActionScaleFactor;
		if (scaleDamping)
			dampingActionScaleFactor = MathTools.clamp(1.0 - (1.0 - minimumDampingScale) * percentError, 0.0, 1.0);
		else
			dampingActionScaleFactor = 1.0;
		this.dampingActionScaleFactor.set(dampingActionScaleFactor);

		double jointSpaceDAction = dampingActionScaleFactor * jointSpaceVelocityGain * -kneePitchJoint.getQd();
		double actuatorSpaceDAction = dampingActionScaleFactor * actuatorSpaceVelocityGain * currentVirtualVelocity;

		double pAction, dAction;

		if (useActuatorSpacePositionControl)
			pAction = actuatorSpacePAction;
		else if (blendPositionError)
			pAction = InterpolationTools.linearInterpolate(jointSpacePAction, actuatorSpacePAction, positionBlendingFactor);
		else
			pAction = jointSpacePAction;

		if (useActuatorSpaceVelocityControl)
			dAction = actuatorSpaceDAction;
		else if (blendVelocityError)
			dAction = InterpolationTools.linearInterpolate(jointSpaceDAction, actuatorSpaceDAction, velocityBlendingFactor);
		else
			dAction = jointSpaceDAction;

		kneePrivilegedPAction.set(pAction);
		kneePrivilegedDAction.set(dAction);

		effectiveKneeStiffness.set(pAction / error);
		effectiveKneeDamping.set(-dAction / kneePitchJoint.getQd());

		return MathTools.clamp(kneePrivilegedPAction.getDoubleValue() + kneePrivilegedDAction.getDoubleValue(), privilegedMaxAcceleration.getDoubleValue());
	}

	private double computeVirtualActuatorLength(double kneePitchAngle)
	{
		double length = Math.pow(thighLength, 2.0) + Math.pow(shinLength, 2.0) + 2.0 * thighLength * shinLength * Math.cos(kneePitchAngle);
		return Math.sqrt(length);
	}

	private double computeVirtualActuatorVelocity(double kneePitchAngle, double kneePitchVelocity)
	{
		double virtualLength = computeVirtualActuatorLength(kneePitchAngle);
		double velocity = -thighLength * shinLength / virtualLength * kneePitchVelocity * Math.sin(kneePitchAngle);
		return velocity;
	}

	public void setKneeAngleState(LegConfigurationType controlType)
	{
		requestedState.set(controlType);
	}

	public LegConfigurationType getCurrentKneeControlState()
	{
		return stateMachine.getCurrentStateEnum();
	}

	public InverseDynamicsCommand<?> getInverseDynamicsCommand()
	{
		return privilegedAccelerationCommand;
	}

	private class StraighteningKneeControlState extends FinishableState<LegConfigurationType>
	{
		private final YoDouble yoStraighteningSpeed;

		private double startingPosition;
		private double previousKneePitchAngle;

		private double timeUntilStraight;
		private double straighteningSpeed;

		private double dwellTime;
		private double desiredPrivilegedPosition;

		private double previousTime;

		public StraighteningKneeControlState(YoDouble straighteningSpeed)
		{
			super(LegConfigurationType.STRAIGHTEN);

			this.yoStraighteningSpeed = straighteningSpeed;
		}

		@Override
		public boolean isDone()
		{
			return getTimeInCurrentState() > (timeUntilStraight + dwellTime);
		}

		@Override
		public void doAction()
		{
			double estimatedDT = estimateDT();
			double currentPosition = kneePitchJoint.getQ();

			if (ONLY_MOVE_PRIV_POS_IF_NOT_BENDING)
			{
				if (currentPosition > previousKneePitchAngle && currentPosition > startingPosition) // the knee is bending
					dwellTime += estimatedDT;
				else
					desiredPrivilegedPosition -= estimatedDT * straighteningSpeed;
			}
			else
			{
				desiredPrivilegedPosition -= estimatedDT * straighteningSpeed;
			}

			kneePitchPrivilegedConfiguration.set(desiredPrivilegedPosition);

			jointSpaceConfigurationGain = straightJointSpacePositionGain.getDoubleValue();
			jointSpaceVelocityGain = straightJointSpaceVelocityGain.getDoubleValue();
			actuatorSpaceConfigurationGain = straightActuatorSpacePositionGain.getDoubleValue();
			actuatorSpaceVelocityGain = straightActuatorSpaceVelocityGain.getDoubleValue();
			maxPositionBlendingFactor = straightMaxPositionBlendingFactor.getDoubleValue();
			maxVelocityBlendingFactor = straightMaxVelocityBlendingFactor.getDoubleValue();
			useActuatorSpacePositionControl = straightUseActuatorSpacePositionControl.getBooleanValue();
			useActuatorSpaceVelocityControl = straightUseActuatorSpaceVelocityControl.getBooleanValue();

			blendPositionError = straightLegGains.getBlendPositionError();
			blendVelocityError = straightLegGains.getBlendVelocityError();

			previousKneePitchAngle = currentPosition;

			if (isDone())
				transitionToDefaultNextState();
		}

		@Override
		public void doTransitionIntoAction()
		{
			startingPosition = kneePitchJoint.getQ();
			previousKneePitchAngle = kneePitchJoint.getQ();

			straighteningSpeed = yoStraighteningSpeed.getDoubleValue();
			timeUntilStraight = (startingPosition - desiredAngle.getDoubleValue()) / straighteningSpeed;
			timeUntilStraight = Math.max(timeUntilStraight, 0.0);

			desiredPrivilegedPosition = startingPosition;

			previousTime = 0.0;
			dwellTime = 0.0;

			if (useBracingAngle.getBooleanValue())
				legControlWeight.set(LegControlWeight.MEDIUM);
		}

		@Override
		public void doTransitionOutOfAction()
		{
		}

		private double estimateDT()
		{
			double currentTime = getTimeInCurrentState();

			double estimatedDT = currentTime - previousTime;
			previousTime = currentTime;

			return estimatedDT;
		}
	}

	private class StraightKneeControlState extends FinishableState<LegConfigurationType>
	{
		public StraightKneeControlState()
		{
			super(LegConfigurationType.STRAIGHT);
		}

		@Override
		public boolean isDone()
		{
			return false;
		}

		@Override
		public void doAction()
		{
			kneePitchPrivilegedConfiguration.set(desiredAngle.getDoubleValue());

			jointSpaceConfigurationGain = straightJointSpacePositionGain.getDoubleValue();
			jointSpaceVelocityGain = straightJointSpaceVelocityGain.getDoubleValue();
			actuatorSpaceConfigurationGain = straightActuatorSpacePositionGain.getDoubleValue();
			actuatorSpaceVelocityGain = straightActuatorSpaceVelocityGain.getDoubleValue();
			maxPositionBlendingFactor = straightMaxPositionBlendingFactor.getDoubleValue();
			maxVelocityBlendingFactor = straightMaxVelocityBlendingFactor.getDoubleValue();
			useActuatorSpacePositionControl = straightUseActuatorSpacePositionControl.getBooleanValue();
			useActuatorSpaceVelocityControl = straightUseActuatorSpaceVelocityControl.getBooleanValue();

			blendPositionError = straightLegGains.getBlendPositionError();
			blendVelocityError = straightLegGains.getBlendVelocityError();
		}

		@Override
		public void doTransitionIntoAction()
		{
		}

		@Override
		public void doTransitionOutOfAction()
		{
		}
	}

	private class BentKneeControlState extends FinishableState<LegConfigurationType>
	{
		public BentKneeControlState()
		{
			super(LegConfigurationType.BENT);
		}

		@Override
		public boolean isDone()
		{
			return false;
		}

		@Override
		public void doAction()
		{
			kneePitchPrivilegedConfiguration.set(kneeMidRangeOfMotion);

			jointSpaceConfigurationGain = bentJointSpacePositionGain.getDoubleValue();
			jointSpaceVelocityGain = bentJointSpaceVelocityGain.getDoubleValue();
			actuatorSpaceConfigurationGain = bentActuatorSpacePositionGain.getDoubleValue();
			actuatorSpaceVelocityGain = bentActuatorSpaceVelocityGain.getDoubleValue();
			maxPositionBlendingFactor = bentMaxPositionBlendingFactor.getDoubleValue();
			maxVelocityBlendingFactor = bentMaxVelocityBlendingFactor.getDoubleValue();
			useActuatorSpacePositionControl = bentUseActuatorSpacePositionControl.getBooleanValue();
			useActuatorSpaceVelocityControl = bentUseActuatorSpaceVelocityControl.getBooleanValue();

			blendPositionError = bentLegGains.getBlendPositionError();
			blendVelocityError = bentLegGains.getBlendVelocityError();
		}

		@Override
		public void doTransitionIntoAction()
		{
			legControlWeight.set(LegControlWeight.LOW);
		}

		@Override
		public void doTransitionOutOfAction()
		{
		}
	}

	private class CollapseKneeControlState extends FinishableState<LegConfigurationType>
	{
		public CollapseKneeControlState()
		{
			super(LegConfigurationType.COLLAPSE);
		}

		@Override
		public boolean isDone()
		{
			return false;
		}

		@Override
		public void doAction()
		{
			double collapsedAngle = desiredFractionOfMidRangeForCollapsed.getDoubleValue() * kneeMidRangeOfMotion;
			double alpha = MathTools.clamp(getTimeInCurrentState() / collapsingDuration.getDoubleValue(), 0.0, 1.0);
			double desiredKneePosition = InterpolationTools.linearInterpolate(desiredAngle.getDoubleValue(), collapsedAngle, alpha);

			kneePitchPrivilegedConfiguration.set(desiredKneePosition);

			jointSpaceConfigurationGain = bentJointSpacePositionGain.getDoubleValue();
			jointSpaceVelocityGain = bentJointSpaceVelocityGain.getDoubleValue();
			actuatorSpaceConfigurationGain = bentActuatorSpacePositionGain.getDoubleValue();
			actuatorSpaceVelocityGain = bentActuatorSpaceVelocityGain.getDoubleValue();
			maxPositionBlendingFactor = bentMaxPositionBlendingFactor.getDoubleValue();
			maxVelocityBlendingFactor = bentMaxVelocityBlendingFactor.getDoubleValue();
			useActuatorSpacePositionControl = bentUseActuatorSpacePositionControl.getBooleanValue();
			useActuatorSpaceVelocityControl = bentUseActuatorSpaceVelocityControl.getBooleanValue();

			blendPositionError = bentLegGains.getBlendPositionError();
			blendVelocityError = bentLegGains.getBlendVelocityError();
			
		}

		@Override
		public void doTransitionIntoAction()
		{
			legControlWeight.set(LegControlWeight.LOW);
		}

		@Override
		public void doTransitionOutOfAction()
		{
		}
	}
}
