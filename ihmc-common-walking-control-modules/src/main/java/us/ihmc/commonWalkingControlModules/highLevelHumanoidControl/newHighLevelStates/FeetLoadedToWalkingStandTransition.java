package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FeetLoadedToWalkingStandTransition extends FeetLoadedTransition
{
   private final FinishableState<HighLevelControllerState> currentState;
   private final HighLevelControllerState nextStateEnum;
   private final YoEnum<HighLevelControllerState> requestedState;

   private final YoBoolean waitForRequest;

   private final YoDouble minimumTimeInState;

   public FeetLoadedToWalkingStandTransition(FinishableState<HighLevelControllerState> currentState, HighLevelControllerState nextStateEnum,
                                             YoEnum<HighLevelControllerState> requestedState, ForceSensorDataHolderReadOnly forceSensorDataHolder, SideDependentList<String> feetForceSensors,
                                             double controlDT, double totalMass, double gravityZ, HighLevelControllerParameters highLevelControllerParameters, YoVariableRegistry parentRegistry)
   {
      super(forceSensorDataHolder, feetForceSensors, controlDT, totalMass, gravityZ, parentRegistry);

      this.currentState = currentState;
      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;

      minimumTimeInState = new YoDouble("minimumTimeLoadingFeet", registry);
      minimumTimeInState.set(highLevelControllerParameters.getMinimumTimeInStandReady());

      this.waitForRequest = new YoBoolean("waitForRequestToTransitionToWalking", registry);
      this.waitForRequest.set(!highLevelControllerParameters.automaticallyTransitionToWalkingWhenReady());
   }

   @Override
   public boolean checkCondition()
   {
      if (!super.checkCondition())
         return false;

      if (currentState.getTimeInCurrentState() < minimumTimeInState.getDoubleValue())
         return false;

      boolean transitionRequested = nextStateEnum.equals(requestedState.getEnumValue());

      if (waitForRequest.getBooleanValue())
         return transitionRequested;
      else
         return true;
   }
}
