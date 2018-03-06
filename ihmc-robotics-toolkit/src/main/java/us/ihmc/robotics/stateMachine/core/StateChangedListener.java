package us.ihmc.robotics.stateMachine.core;

public interface StateChangedListener<K>
{
   void stateChanged(K from, K to);
}
