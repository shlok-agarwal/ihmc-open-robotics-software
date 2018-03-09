package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.AnkleIKSolver;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator.ToeOffCalculator;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.providers.YoVelocityProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateMachineTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FootControlModule
{
   private final YoVariableRegistry registry;
   private final ContactablePlaneBody contactableFoot;

   public enum ConstraintType
   {
      FULL, TOES, SWING, MOVE_VIA_WAYPOINTS, TOUCHDOWN;

      public boolean isLoadBearing()
      {
         switch (this)
         {
         case FULL:
         case TOES:
            return true;
         default:
            return false;
         }
      }
   }

   private static final double coefficientOfFriction = 0.8;

   private final GenericStateMachine<ConstraintType, AbstractFootControlState> stateMachine;
   private final YoEnum<ConstraintType> requestedState;
   private final EnumMap<ConstraintType, boolean[]> contactStatesMap = new EnumMap<ConstraintType, boolean[]>(ConstraintType.class);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final RobotSide robotSide;

   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;

   private final SwingState swingState;
   private final MoveViaWaypointsState moveViaWaypointsState;
   private final TouchDownState touchdownState;
   private final OnToesState onToesState;
   private final SupportState supportState;

   private final YoDouble footLoadThresholdToHoldPosition;

   private final FootControlHelper footControlHelper;

   private final YoBoolean requestExploration;
   private final YoBoolean resetFootPolygon;
   
   private final FramePose3D desiredPose = new FramePose3D();
   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final UnloadedAnkleControlModule ankleControlModule;

   public FootControlModule(RobotSide robotSide, ToeOffCalculator toeOffCalculator, WalkingControllerParameters walkingControllerParameters,
                            PIDSE3GainsReadOnly swingFootControlGains, PIDSE3GainsReadOnly holdPositionFootControlGains,
                            PIDSE3GainsReadOnly toeOffFootControlGains, HighLevelHumanoidControllerToolbox controllerToolbox,
                            ExplorationParameters explorationParameters, YoVariableRegistry parentRegistry)
   {
      contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      controllerToolbox.setFootContactCoefficientOfFriction(robotSide, coefficientOfFriction);
      controllerToolbox.setFootContactStateFullyConstrained(robotSide);

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Foot";
      registry = new YoVariableRegistry(sidePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      footControlHelper = new FootControlHelper(robotSide, walkingControllerParameters, controllerToolbox, explorationParameters, registry);

      this.controllerToolbox = controllerToolbox;
      this.robotSide = robotSide;

      footLoadThresholdToHoldPosition = new YoDouble("footLoadThresholdToHoldPosition", registry);
      footLoadThresholdToHoldPosition.set(0.2);
      
      legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();

      // set up states and state machine
      YoDouble time = controllerToolbox.getYoTime();
      stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "SwitchTime", ConstraintType.class, time, registry);
      requestedState = YoEnum.create(namePrefix + "RequestedState", "", ConstraintType.class, registry, true);
      requestedState.set(null);

      setupContactStatesMap();

      Vector3D touchdownVelocity = new Vector3D(0.0, 0.0, swingTrajectoryParameters.getDesiredTouchdownVelocity());
      YoFrameVector yoTouchdownVelocity = new YoFrameVector(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), registry);
      yoTouchdownVelocity.set(touchdownVelocity);

      Vector3D touchdownAcceleration = new Vector3D(0.0, 0.0, swingTrajectoryParameters.getDesiredTouchdownAcceleration());
      YoFrameVector yoTouchdownAcceleration = new YoFrameVector(namePrefix + "TouchdownAcceleration", ReferenceFrame.getWorldFrame(), registry);
      yoTouchdownAcceleration.set(touchdownAcceleration);

      YoVelocityProvider touchdownVelocityProvider = new YoVelocityProvider(yoTouchdownVelocity);
      YoVelocityProvider touchdownAccelerationProvider = new YoVelocityProvider(yoTouchdownAcceleration);

      List<AbstractFootControlState> states = new ArrayList<AbstractFootControlState>();

      onToesState = new OnToesState(footControlHelper, toeOffCalculator, toeOffFootControlGains, registry);
      states.add(onToesState);

      supportState = new SupportState(footControlHelper, holdPositionFootControlGains, registry);
      states.add(supportState);

      swingState = new SwingState(footControlHelper, yoTouchdownVelocity, yoTouchdownAcceleration, swingFootControlGains, registry);
      states.add(swingState);
      
      touchdownState = new TouchDownState(footControlHelper, swingFootControlGains, registry);
      states.add(touchdownState);

      moveViaWaypointsState = new MoveViaWaypointsState(footControlHelper, touchdownVelocityProvider, touchdownAccelerationProvider, swingFootControlGains, registry);
      states.add(moveViaWaypointsState);

      setupStateMachine(states);

      requestExploration = new YoBoolean(namePrefix + "RequestExploration", registry);
      resetFootPolygon = new YoBoolean(namePrefix + "ResetFootPolygon", registry);

      AnkleIKSolver ankleIKSolver = walkingControllerParameters.getAnkleIKSolver();
      if (ankleIKSolver != null)
      {
         FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
         ankleControlModule = new UnloadedAnkleControlModule(fullRobotModel, robotSide, ankleIKSolver, registry);
      }
      else
      {
         ankleControlModule = null;
      }
   }

   private void setupContactStatesMap()
   {
      boolean[] falses = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(falses, false);
      boolean[] trues = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(trues, true);

      contactStatesMap.put(ConstraintType.SWING, falses);
      contactStatesMap.put(ConstraintType.MOVE_VIA_WAYPOINTS, falses);
      contactStatesMap.put(ConstraintType.FULL, trues);
      contactStatesMap.put(ConstraintType.TOES, getOnEdgeContactPointStates(contactableFoot, ConstraintType.TOES));
      contactStatesMap.put(ConstraintType.TOUCHDOWN, falses);
   }

   private void setupStateMachine(List<AbstractFootControlState> states)
   {
      // TODO Clean that up (Sylvain)
      for (AbstractFootControlState fromState : states)
      {
         for (AbstractFootControlState toState : states)
         {
            StateMachineTools.addRequestedStateTransition(requestedState, false, fromState, toState);
         }
      }
      
      for (AbstractFootControlState state : states)
      {
         stateMachine.addState(state);
      }
      stateMachine.setCurrentState(ConstraintType.FULL);
   }

   public void setWeights(Vector3DReadOnly loadedFootAngularWeight, Vector3DReadOnly loadedFootLinearWeight, Vector3DReadOnly footAngularWeight,
                          Vector3DReadOnly footLinearWeight)
   {
      swingState.setWeights(footAngularWeight, footLinearWeight);
      moveViaWaypointsState.setWeights(footAngularWeight, footLinearWeight);
      touchdownState.setWeights(footAngularWeight, footLinearWeight);
      onToesState.setWeights(loadedFootAngularWeight, loadedFootLinearWeight);
      supportState.setWeights(loadedFootAngularWeight, loadedFootLinearWeight);
   }

   public void setAdjustedFootstepAndTime(Footstep adjustedFootstep, double swingTime)
   {
      swingState.setAdjustedFootstepAndTime(adjustedFootstep, swingTime);
   }

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (stateMachine.getCurrentState() == moveViaWaypointsState)
         moveViaWaypointsState.requestTouchdownForDisturbanceRecovery();
   }
   
   public void requestTouchdown()
   {
    if(stateMachine.isCurrentState(ConstraintType.SWING))
    {
       setContactState(ConstraintType.TOUCHDOWN);
       swingState.getDesireds(desiredPose, desiredLinearVelocity, desiredAngularVelocity);
       touchdownState.initialize(desiredPose, desiredLinearVelocity, desiredAngularVelocity);
    }
   }

   public void requestStopTrajectoryIfPossible()
   {
      if (stateMachine.getCurrentState() == moveViaWaypointsState)
         moveViaWaypointsState.requestStopTrajectory();
   }

   public void setContactState(ConstraintType constraintType)
   {
      setContactState(constraintType, null);
   }

   public void setContactState(ConstraintType constraintType, FrameVector3D normalContactVector)
   {
      if (constraintType == ConstraintType.FULL)
      {
         footControlHelper.setFullyConstrainedNormalContactVector(normalContactVector);
      }

      controllerToolbox.setFootContactState(robotSide, contactStatesMap.get(constraintType), normalContactVector);

      if (getCurrentConstraintType() == constraintType) // Use resetCurrentState() for such case
         return;

      requestedState.set(constraintType);
   }

   public ConstraintType getCurrentConstraintType()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public void initialize()
   {
      stateMachine.setCurrentState(ConstraintType.FULL);
   }

   public void doControl()
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.resetSwingParameters();
      }

      footControlHelper.update();

      if (resetFootPolygon.getBooleanValue())
      {
         resetFootPolygon();
      }

      if (requestExploration.getBooleanValue())
      {
         requestExploration();
      }
      
      stateMachine.checkTransitionConditions();

      if (!isInFlatSupportState() && footControlHelper.getPartialFootholdControlModule() != null)
         footControlHelper.getPartialFootholdControlModule().reset();
      

      stateMachine.doAction();

      if (ankleControlModule != null)
      {
         ankleControlModule.compute(stateMachine.getCurrentState());
      }
   }

   // Used to restart the current state reseting the current state time
   public void resetCurrentState()
   {
      stateMachine.setCurrentState(getCurrentConstraintType());
   }
   
   public boolean isInFlatSupportState()
   {
      ConstraintType currentConstraintType = getCurrentConstraintType();
      return currentConstraintType == ConstraintType.FULL;
   }

   public boolean isInToeOff()
   {
      return getCurrentConstraintType() == ConstraintType.TOES;
   }

   public void setUsePointContactInToeOff(boolean usePointContact)
   {
      onToesState.setUsePointContact(usePointContact);
   }

   private boolean[] getOnEdgeContactPointStates(ContactablePlaneBody contactableBody, ConstraintType constraintType)
   {
      FrameVector3D direction = new FrameVector3D(contactableBody.getFrameAfterParentJoint(), 1.0, 0.0, 0.0);

      int[] indexOfPointsInContact = DesiredFootstepCalculatorTools.findMaximumPointIndexesInDirection(contactableBody.getContactPointsCopy(), direction, 2);

      boolean[] contactPointStates = new boolean[contactableBody.getTotalNumberOfContactPoints()];

      for (int i = 0; i < indexOfPointsInContact.length; i++)
      {
         contactPointStates[indexOfPointsInContact[i]] = true;
      }

      return contactPointStates;
   }

   public void updateLegSingularityModule()
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.update();
      }
   }

   public void correctCoMHeightTrajectoryForSingularityAvoidance(FrameVector2D comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect,
                                                                 double zCurrent, ReferenceFrame pelvisZUpFrame)
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForSingularityAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent,
                                                                                                               pelvisZUpFrame, getCurrentConstraintType());
      }
   }

   public void correctCoMHeightTrajectoryForCollapseAvoidance(FrameVector2D comXYVelocity, CoMHeightTimeDerivativesData comHeightDataToCorrect, double zCurrent,
                                                              ReferenceFrame pelvisZUpFrame, double footLoadPercentage)
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForCollapseAvoidance(comXYVelocity, comHeightDataToCorrect, zCurrent,
                                                                                                            pelvisZUpFrame, footLoadPercentage,
                                                                                                            getCurrentConstraintType());
      }
   }

   public void correctCoMHeightTrajectoryForUnreachableFootStep(CoMHeightTimeDerivativesData comHeightDataToCorrect)
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.correctCoMHeightTrajectoryForUnreachableFootStep(comHeightDataToCorrect,
                                                                                                              getCurrentConstraintType());
      }
   }

   public void setFootstep(Footstep footstep, double swingTime, double touchdownTime)
   {
      swingState.setFootstep(footstep, swingTime);
      touchdownState.setTouchdownDuration(touchdownTime);
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command)
   {
      moveViaWaypointsState.handleFootTrajectoryCommand(command);
   }

   public void resetHeightCorrectionParametersForSingularityAvoidance()
   {
      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.resetHeightCorrectionParameters();
      }
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(double speedUpFactor)
   {
      return swingState.requestSwingSpeedUp(speedUpFactor);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());
      if (ankleControlModule != null)
      {
         inverseDynamicsCommandList.addCommand(ankleControlModule.getInverseDynamicsCommand());
      }
      return inverseDynamicsCommandList;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public JointDesiredOutputListReadOnly getJointDesiredData()
   {
      if (ankleControlModule != null)
      {
         return ankleControlModule.getJointDesiredOutputList();
      }
      return null;
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (ConstraintType constraintType : ConstraintType.values())
      {
         AbstractFootControlState state = stateMachine.getState(constraintType);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }
   
   public boolean isInTouchdown()
   {
      return getCurrentConstraintType().equals(ConstraintType.TOUCHDOWN);
   }

   public void initializeFootExploration()
   {
      supportState.requestFootholdExploration();
   }

   public boolean isFootToeingOffSlipping()
   {
      if (getCurrentConstraintType() != ConstraintType.TOES)
         return false;
      if (footControlHelper.getToeSlippingDetector() == null)
         return false;
      return footControlHelper.getToeSlippingDetector().isToeSlipping();
   }

   private void requestExploration()
   {
      if (!isInFlatSupportState()) return;
      requestExploration.set(false);
      initializeFootExploration();
   }

   private void resetFootPolygon()
   {
      if (!isInFlatSupportState()) return;
      resetFootPolygon.set(false);
      if (footControlHelper.getPartialFootholdControlModule() != null)
      {
         footControlHelper.getPartialFootholdControlModule().reset();
      }
      controllerToolbox.resetFootSupportPolygon(robotSide);
   }
}