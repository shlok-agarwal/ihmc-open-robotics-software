package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.ControlledBodiesCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedSteppingEventPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedSteppingStatePacket;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedPreplannedStepStream;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedStepStreamMultiplexer;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPreplannedStepInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedSoleWaypointInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedXGaitSettingsInputProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineYoVariableTrigger;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedSteppingState implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedXGaitSettingsInputProvider xGaitSettingsProvider;
   private final QuadrupedPreplannedStepInputProvider preplannedStepProvider;
   private final QuadrupedPlanarVelocityInputProvider planarVelocityProvider;
   private final QuadrupedSoleWaypointInputProvider soleWaypointInputProvider;

   private final QuadrupedPreplannedStepStream preplannedStepStream;
   private final QuadrupedXGaitStepStream xGaitStepStream;
   private final QuadrupedStepStreamMultiplexer<QuadrupedSteppingStateEnum> stepStreamMultiplexer;

   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedForceControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;

   private final YoEnum<QuadrupedSteppingRequestedEvent> lastEvent = new YoEnum<>("lastSteppingEvent", registry, QuadrupedSteppingRequestedEvent.class);
   private final FiniteStateMachine<QuadrupedSteppingStateEnum, ControllerEvent, QuadrupedController> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedSteppingRequestedEvent> stepTrigger;

   private final QuadrupedSteppingStatePacket quadrupedSteppingStatePacket;
   private final AtomicReference<QuadrupedSteppingRequestedEvent> requestedEvent = new AtomicReference<>();

   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
   private final ControlledBodiesCommand controlledBodiesCommand;
   private final WholeBodyControllerCore controllerCore;

   public QuadrupedSteppingState(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedForceControllerToolbox controllerToolbox,
                                 QuadrupedControlManagerFactory controlManagerFactory, YoVariableRegistry parentRegistry)
   {
      this.runtimeEnvironment = runtimeEnvironment;
      this.controllerToolbox = controllerToolbox;
      this.controlManagerFactory = controlManagerFactory;

      feetManager = controlManagerFactory.getOrCreateFeetManager();
      balanceManager = controlManagerFactory.getOrCreateBalanceManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();

      FullQuadrupedRobotModel fullRobotModel = runtimeEnvironment.getFullRobotModel();
      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(runtimeEnvironment.getControlDT(), runtimeEnvironment.getGravity(),
                                                                                       fullRobotModel.getRootJoint(), fullRobotModel.getControllableOneDoFJoints(),
                                                                                       controllerToolbox.getReferenceFrames().getCenterOfMassFrame(),
                                                                                       runtimeEnvironment.getControllerCoreOptimizationSettings(),
                                                                                       runtimeEnvironment.getGraphicsListRegistry(), registry);

      RigidBody[] controlledBodies = {fullRobotModel.getBody(), fullRobotModel.getFoot(RobotQuadrant.FRONT_LEFT), fullRobotModel.getFoot(RobotQuadrant.FRONT_RIGHT),
            fullRobotModel.getFoot(RobotQuadrant.HIND_LEFT), fullRobotModel.getFoot(RobotQuadrant.HIND_RIGHT) };
      controlCoreToolbox.setupForVirtualModelControlSolver(fullRobotModel.getBody(), controlledBodies, controllerToolbox.getContactablePlaneBodies());
      FeedbackControlCommandList feedbackTemplate = controlManagerFactory.createFeedbackControlTemplate();
      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, feedbackTemplate, runtimeEnvironment.getJointDesiredOutputList(),
                                                   registry);

      // Initialize input providers.
      xGaitSettingsProvider = new QuadrupedXGaitSettingsInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      preplannedStepProvider = new QuadrupedPreplannedStepInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      planarVelocityProvider = new QuadrupedPlanarVelocityInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      soleWaypointInputProvider = new QuadrupedSoleWaypointInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);

      // Initialize input step streams.
      xGaitStepStream = new QuadrupedXGaitStepStream(planarVelocityProvider, xGaitSettingsProvider, controllerToolbox.getReferenceFrames(),
                                                     runtimeEnvironment.getControlDT(), runtimeEnvironment.getRobotTimestamp(), registry);
      preplannedStepStream = new QuadrupedPreplannedStepStream(preplannedStepProvider, controllerToolbox.getReferenceFrames(),
                                                               runtimeEnvironment.getRobotTimestamp(), registry);
      stepStreamMultiplexer = new QuadrupedStepStreamMultiplexer<>(QuadrupedSteppingStateEnum.class, registry);
      stepStreamMultiplexer.addStepStream(QuadrupedSteppingStateEnum.XGAIT, xGaitStepStream);
      stepStreamMultiplexer.addStepStream(QuadrupedSteppingStateEnum.STEP, preplannedStepStream);
      stepStreamMultiplexer.selectStepStream(QuadrupedSteppingStateEnum.XGAIT);


      GlobalDataProducer globalDataProducer = runtimeEnvironment.getGlobalDataProducer();

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedSteppingEventPacket.class, new PacketConsumer<QuadrupedSteppingEventPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedSteppingEventPacket packet)
            {
               requestedEvent.set(packet.get());
            }
         });
      }

      this.quadrupedSteppingStatePacket = new QuadrupedSteppingStatePacket();

      controlledBodiesCommand = registerControlledBodies();

      this.stateMachine = buildStateMachine(runtimeEnvironment);
      this.stepTrigger = new FiniteStateMachineYoVariableTrigger<>(stateMachine, "stepTrigger", registry, QuadrupedSteppingRequestedEvent.class);

      parentRegistry.addChild(registry);
   }

   private FiniteStateMachine<QuadrupedSteppingStateEnum, ControllerEvent, QuadrupedController> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment)
   {
      // Initialize controllers.
      final QuadrupedController standController = new QuadrupedStandController(controllerToolbox, controlManagerFactory, registry);
      final QuadrupedStepController stepController = new QuadrupedStepController(controllerToolbox, controlManagerFactory, stepStreamMultiplexer, registry);
      final QuadrupedController soleWaypointController = new QuadrupedForceBasedSoleWaypointController(controllerToolbox, controlManagerFactory,
                                                                                                       soleWaypointInputProvider, registry);

      FiniteStateMachineBuilder<QuadrupedSteppingStateEnum, ControllerEvent, QuadrupedController> builder = new FiniteStateMachineBuilder<>(QuadrupedSteppingStateEnum.class,
                                                                                                                                              ControllerEvent.class, "steppingState", registry);

      builder.addState(QuadrupedSteppingStateEnum.STAND, standController);
      builder.addState(QuadrupedSteppingStateEnum.STEP, stepController);
      builder.addState(QuadrupedSteppingStateEnum.XGAIT, stepController);
      builder.addState(QuadrupedSteppingStateEnum.SOLE_WAYPOINT, soleWaypointController);

      // Add automatic transitions that lead into the stand state.
      builder.addTransition(ControllerEvent.DONE, QuadrupedSteppingStateEnum.STEP, QuadrupedSteppingStateEnum.STAND);
      builder.addTransition(ControllerEvent.DONE, QuadrupedSteppingStateEnum.XGAIT, QuadrupedSteppingStateEnum.STAND);

      // Sole Waypoint events
      builder.addTransition(QuadrupedSteppingRequestedEvent.class, QuadrupedSteppingRequestedEvent.REQUEST_SOLE_WAYPOINT,
                            QuadrupedSteppingStateEnum.STAND, QuadrupedSteppingStateEnum.SOLE_WAYPOINT);
      builder.addTransition(ControllerEvent.DONE, QuadrupedSteppingStateEnum.SOLE_WAYPOINT, QuadrupedSteppingStateEnum.STAND);
      builder.addTransition(ControllerEvent.FAIL, QuadrupedSteppingStateEnum.SOLE_WAYPOINT, QuadrupedSteppingStateEnum.STAND);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedSteppingRequestedEvent.class, QuadrupedSteppingRequestedEvent.REQUEST_STEP,
                            QuadrupedSteppingStateEnum.STAND, QuadrupedSteppingStateEnum.STEP);
      builder.addTransition(QuadrupedSteppingRequestedEvent.class, QuadrupedSteppingRequestedEvent.REQUEST_XGAIT,
                            QuadrupedSteppingStateEnum.STAND, QuadrupedSteppingStateEnum.XGAIT);

      // Callbacks functions.
      Runnable standToXGaitCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepStreamMultiplexer.selectStepStream(QuadrupedSteppingStateEnum.XGAIT);
         }
      };
      builder.addCallback(QuadrupedSteppingRequestedEvent.class, QuadrupedSteppingRequestedEvent.REQUEST_XGAIT,
                          QuadrupedSteppingStateEnum.STAND, standToXGaitCallback);

      Runnable xGaitToStandCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepController.halt();
         }
      };
      builder.addCallback(QuadrupedSteppingRequestedEvent.class, QuadrupedSteppingRequestedEvent.REQUEST_STAND,
                          QuadrupedSteppingStateEnum.XGAIT, xGaitToStandCallback);

      Runnable standToStepCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepStreamMultiplexer.selectStepStream(QuadrupedSteppingStateEnum.STEP);
         }
      };
      builder.addCallback(QuadrupedSteppingRequestedEvent.class, QuadrupedSteppingRequestedEvent.REQUEST_STEP, QuadrupedSteppingStateEnum.STAND,
                          standToStepCallback);

      Runnable stepToStandCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepController.halt();
         }
      };
      builder.addCallback(QuadrupedSteppingRequestedEvent.class, QuadrupedSteppingRequestedEvent.REQUEST_STAND, QuadrupedSteppingStateEnum.STEP,
                          stepToStandCallback);

      return builder.build(QuadrupedSteppingStateEnum.STAND);
   }

   public ControlledBodiesCommand registerControlledBodies()
   {
      FullQuadrupedRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      ControlledBodiesCommand command = new ControlledBodiesCommand();
      command.addBodyToControl(fullRobotModel.getFoot(RobotQuadrant.FRONT_RIGHT));
      command.addBodyToControl(fullRobotModel.getFoot(RobotQuadrant.FRONT_LEFT));
      command.addBodyToControl(fullRobotModel.getFoot(RobotQuadrant.HIND_RIGHT));
      command.addBodyToControl(fullRobotModel.getFoot(RobotQuadrant.HIND_LEFT));
      command.addBodyToControl(fullRobotModel.getBody());

      return command;
   }

   @Override
   public void onEntry()
   {

   }

   @Override
   public ControllerEvent process()
   {
      QuadrupedSteppingRequestedEvent reqEvent = requestedEvent.getAndSet(null);
      if (reqEvent != null)
      {
         lastEvent.set(reqEvent);
         stateMachine.trigger(QuadrupedSteppingRequestedEvent.class, reqEvent);
      }

      if (preplannedStepProvider.isStepPlanAvailable())
      {
         if (stateMachine.getCurrentStateEnum() == QuadrupedSteppingStateEnum.STAND)
         {
            // trigger step event if preplanned steps are available in stand state
            lastEvent.set(QuadrupedSteppingRequestedEvent.REQUEST_STEP);
            stateMachine.trigger(QuadrupedSteppingRequestedEvent.class, QuadrupedSteppingRequestedEvent.REQUEST_STEP);
         }
      }

      // update controller state machine
      stateMachine.process();

      // Send state information
      quadrupedSteppingStatePacket.set(stateMachine.getCurrentStateEnum());

      if (runtimeEnvironment.getGlobalDataProducer() != null)
      {
         runtimeEnvironment.getGlobalDataProducer().queueDataToSend(quadrupedSteppingStatePacket);
      }

      submitControllerCoreCommands();

      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();

      return null;
   }

   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandPool = new RecyclingArrayList<>(4, PlaneContactStateCommand.class);

   private void submitControllerCoreCommands()
   {
      planeContactStateCommandPool.clear();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerCoreCommand.addFeedbackControlCommand(feetManager.getFeedbackControlCommand(robotQuadrant));
         controllerCoreCommand.addVirtualModelControlCommand(feetManager.getInverseDynamicsCommand(robotQuadrant));

         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotQuadrant);
         PlaneContactStateCommand planeContactStateCommand = planeContactStateCommandPool.add();
         contactState.getPlaneContactStateCommand(planeContactStateCommand);
         //planeContactStateCommand.setUseHighCoPDamping(false);
         controllerCoreCommand.addVirtualModelControlCommand(planeContactStateCommand);
      }

      controllerCoreCommand.addFeedbackControlCommand(bodyOrientationManager.getFeedbackControlCommand());
      controllerCoreCommand.addVirtualModelControlCommand(bodyOrientationManager.getInverseDynamicsCommand());

      controllerCoreCommand.addVirtualModelControlCommand(controlledBodiesCommand);

      controllerCoreCommand.addVirtualModelControlCommand(balanceManager.getInverseDynamicsCommand());
   }

   @Override
   public void onExit()
   {

   }
}
