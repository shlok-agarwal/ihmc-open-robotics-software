package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionCondition;

public class StartFlamingoCondition implements StateTransitionCondition
{
   private final RobotSide transferToSide;
   private final WalkingMessageHandler walkingMessageHandler;

   public StartFlamingoCondition(RobotSide transferToSide, WalkingMessageHandler walkingMessageHandler)
   {
      this.transferToSide = transferToSide;
      this.walkingMessageHandler = walkingMessageHandler;
   }

   @Override
   public boolean checkCondition()
   {
      return walkingMessageHandler.hasFootTrajectoryForFlamingoStance(transferToSide.getOppositeSide());
   }
}