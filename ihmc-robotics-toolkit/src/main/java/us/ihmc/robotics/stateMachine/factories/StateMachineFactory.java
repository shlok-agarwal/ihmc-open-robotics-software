package us.ihmc.robotics.stateMachine.factories;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateMachineClock;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.extra.StateTransitionCondition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class StateMachineFactory<K extends Enum<K>, S extends State>
{
   private String namePrefix;
   private YoVariableRegistry registry;

   protected final Map<K, S> states;
   private final Map<K, StateTransitionList<K>> stateTransitions;
   private final List<StateChangedListener<K>> stateChangedListeners = new ArrayList<>();
   private StateMachineClock clock = StateMachineClock.createDummyClock();

   public StateMachineFactory(Class<K> keyType)
   {
      states = new EnumMap<>(keyType);
      stateTransitions = new EnumMap<>(keyType);
   }

   public StateMachineFactory<K, S> setNamePrefix(String namePrefix)
   {
      this.namePrefix = namePrefix;
      return this;
   }

   public StateMachineFactory<K, S> setRegistry(YoVariableRegistry registry)
   {
      this.registry = registry;
      return this;
   }

   public StateMachineFactory<K, S> buildClock(DoubleProvider timeProvider)
   {
      clock = StateMachineClock.createClock(timeProvider);
      return this;
   }

   public StateMachineFactory<K, S> buildYoClock(DoubleProvider timeProvider)
   {
      if (namePrefix == null || registry == null)
         throw new RuntimeException("The namePrefix and registry fields have to be set in order to create yo-variables.");
      clock = StateMachineClock.createYoClock(timeProvider, namePrefix, registry);
      return this;
   }

   public StateMachineFactory<K, S> addState(K key, S state)
   {
      S oldState = states.put(key, state);
      if (oldState != null)
         PrintTools.warn("The state " + oldState.getClass().getSimpleName() + " at the key " + key + " has been replaced with "
               + state.getClass().getSimpleName());
      return this;
   }

   public boolean isStateRegistered(K stateKey)
   {
      return states.containsKey(stateKey);
   }

   public StateMachineFactory<K, S> addRequestedTransition(K from, YoEnum<K> requestedState)
   {
      return addTransition(from, new StateTransition<K>()
      {
         @Override
         public K isTransitionRequested(double timeInCurrentState)
         {
            K nextStateKey = requestedState.getEnumValue();
            if (nextStateKey != null)
               requestedState.set(null);
            return nextStateKey;
         }
      });
   }

   public StateMachineFactory<K, S> addRequestedTransition(K from, K to, YoEnum<K> requestedState)
   {
      return addTransition(from, to, new StateTransitionCondition()
      {
         @Override
         public boolean testCondition(double timeInCurrentState)
         {
            K requestedStateKey = requestedState.getEnumValue();

            if (requestedStateKey != null && to == requestedStateKey)
            {
               requestedState.set(null);
               return true;
            }
            return false;
         }
      });
   }

   public StateMachineFactory<K, S> addTimeBasedTransition(K from, K to, double durationBeforeTransition)
   {
      return addTransition(from, to, timeInCurrentState -> durationBeforeTransition <= timeInCurrentState);
   }

   public StateMachineFactory<K, S> addTimeBasedTransition(K from, K to, DoubleProvider durationBeforeTransition)
   {
      return addTransition(from, to, timeInCurrentState -> durationBeforeTransition.getValue() <= timeInCurrentState);
   }

   public StateMachineFactory<K, S> addTransition(Iterable<? extends K> froms, K to, StateTransitionCondition condition)
   {
      froms.forEach(from -> addTransition(from, to, condition));
      return this;
   }

   public StateMachineFactory<K, S> addTransition(K from, K to, StateTransitionCondition condition)
   {
      return addTransition(from, timeInCurrentState -> condition.testCondition(timeInCurrentState) ? to : null);
   }

   public StateMachineFactory<K, S> addTransition(K from, StateTransition<K> stateTransition)
   {
      StateTransitionList<K> transitions = stateTransitions.get(from);
      if (transitions == null)
      {
         transitions = new StateTransitionList<>();
         stateTransitions.put(from, transitions);
      }
      transitions.transitions.add(stateTransition);
      return this;
   }

   public StateMachineFactory<K, S> addStateChangedListener(StateChangedListener<K> stateChangeListener)
   {
      stateChangedListeners.add(stateChangeListener);
      return this;
   }

   public StateMachine<K, S> build(K initialStateKey)
   {
      return new StateMachine<K, S>(initialStateKey, states, stateTransitions, stateChangedListeners, clock, namePrefix, registry);
   }

   private static class StateTransitionList<K> implements StateTransition<K>
   {
      private final List<StateTransition<K>> transitions = new ArrayList<>();

      @Override
      public K isTransitionRequested(double timeInCurrentState)
      {
         for (int i = 0; i < transitions.size(); i++)
         {
            K transitionRequest = transitions.get(i).isTransitionRequested(timeInCurrentState);
            if (transitionRequest != null)
               return transitionRequest;
         }
         return null;
      }
   }
}
