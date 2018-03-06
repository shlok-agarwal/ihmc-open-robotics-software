package us.ihmc.robotics.stateMachine.core;

public interface State
{
   /**
    * Called when the state machine is transitioning into this state.
    */
   void onEntry();

   /**
    * Called regularly by the state machine to allow the state to do any time-dependent internal
    * processing.
    */
   void doAction(double timeInState);

   /**
    * Called when the state machine is transitioning out of this state.
    */
   void onExit();
}
