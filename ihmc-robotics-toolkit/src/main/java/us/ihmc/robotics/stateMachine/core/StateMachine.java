package us.ihmc.robotics.stateMachine.core;

import java.util.List;
import java.util.Map;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class StateMachine<K extends Enum<K>, S extends State>
{
   private final K initialStateKey;
   private final Map<K, S> states;
   private final Map<K, ? extends StateTransition<K>> stateTransitions;
   private final List<StateChangedListener<K>> stateChangedListeners;
   private final StateMachineClock clock;

   private final YoEnum<K> currentStateKey;

   public StateMachine(K initialStateKey, Map<K, S> states, Map<K, ? extends StateTransition<K>> stateTransitions,
                       List<StateChangedListener<K>> stateChangedListeners, StateMachineClock clock, String namePrefix, YoVariableRegistry registry)
   {
      this.initialStateKey = initialStateKey;
      this.states = states;
      this.stateTransitions = stateTransitions;
      this.stateChangedListeners = stateChangedListeners;
      this.clock = clock;

      currentStateKey = new YoEnum<>(namePrefix, "", registry, initialStateKey.getDeclaringClass(), true);
      currentStateKey.set(null);
   }

   public void addStateChangedListener(StateChangedListener<K> listener)
   {
      stateChangedListeners.add(listener);
   }

   public void reset()
   {
      performTransition(initialStateKey);
   }

   public void doControlAndTransition()
   {
      doControl();
      doTransitions();
   }

   public void doControl()
   {
      if (currentStateKey.getEnumValue() == null)
         reset();

      S currentState = getState(currentStateKey.getEnumValue());

      if (currentState == null)
         throw new RuntimeException("There is no state associated with the key: " + currentStateKey);

      currentState.doAction(clock.getTimeInCurrentState());
   }

   public void doTransitions()
   {
      StateTransition<K> stateTransition = stateTransitions.get(currentStateKey.getEnumValue());

      if (stateTransition == null)
         return;

      K nextStateKey = stateTransition.isTransitionRequested(clock.getTimeInCurrentState());

      if (nextStateKey == null)
         return;

      performTransition(nextStateKey);
   }

   private void performTransition(K nextStateKey)
   {
      if (currentStateKey.getEnumValue() != null)
      {
         S currentState = states.get(currentStateKey.getEnumValue());
         if (currentState != null)
            currentState.onExit();
      }
      setCurrentState(nextStateKey);
   }

   public void setCurrentState(K nextStateKey)
   {
      S nextState = getState(nextStateKey);

      if (stateChangedListeners != null)
      {
         for (int i = 0; i < stateChangedListeners.size(); i++)
            stateChangedListeners.get(i).stateChanged(currentStateKey.getEnumValue(), nextStateKey);
      }

      clock.notifyStateChanged();

      nextState.onEntry();
      currentStateKey.set(nextStateKey);
   }

   public K getCurrentStateKey()
   {
      return currentStateKey.getEnumValue();
   }

   public S getCurrentState()
   {
      return getState(currentStateKey.getEnumValue());
   }

   public S getState(K stateKey)
   {
      S currentState = states.get(stateKey);

      if (currentState == null)
         throw new RuntimeException("There is no state associated with the key: " + stateKey);

      return currentState;
   }

   public double getTimeInCurrentState()
   {
      return clock.getTimeInCurrentState();
   }

   public double getTimeOfLastStateChange()
   {
      return clock.getTimeOfLastStateChange();
   }

   public boolean isCurrentStateTerminal()
   {
      return stateTransitions.containsKey(currentStateKey.getEnumValue());
   }
}
