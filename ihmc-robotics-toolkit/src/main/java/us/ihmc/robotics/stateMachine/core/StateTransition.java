package us.ihmc.robotics.stateMachine.core;

public interface StateTransition<K>
{
   K isTransitionRequested(double timeInCurrentState);
}
