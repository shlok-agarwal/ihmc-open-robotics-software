package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.NewVirtualModelControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.virtualModelControl.NewVirtualModelControlSolution;
import us.ihmc.commonWalkingControlModules.virtualModelControl.NewVirtualModelController;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class NewWholeBodyVirtualModelControlSolver
{
   private static final boolean USE_LIMITED_JOINT_TORQUES = true;
   private static final boolean USE_CONTACT_FORCE_QP = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final VirtualModelControlOptimizationControlModule optimizationControlModule;
   private final NewVirtualModelController virtualModelController;

   private NewVirtualModelControlSolution virtualModelControlSolution = new NewVirtualModelControlSolution();

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final WrenchVisualizer wrenchVisualizer;

   private final FloatingInverseDynamicsJoint rootJoint;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final ReferenceFrame centerOfMassFrame;

   private final PoseReferenceFrame controlFrame = new PoseReferenceFrame("controlFrame", worldFrame);

   private final OneDoFJoint[] controlledOneDoFJoints;
   private final RigidBody controlRootBody;

   private final Wrench tempExternalWrench = new Wrench();
   private final DenseMatrix64F tempSelectionMatrix = new DenseMatrix64F();

   private final List<RigidBody> controlledBodies;

   private final YoFrameVector yoDesiredMomentumRateLinear;
   private final YoFrameVector yoAchievedMomentumRateLinear;
   private final YoFrameVector yoDesiredMomentumRateAngular;
   private final YoFrameVector yoAchievedMomentumRateAngular;
   private final FrameVector3D achievedMomentumRateLinear = new FrameVector3D();
   private final Map<OneDoFJoint, RateLimitedYoVariable> jointTorqueSolutions = new HashMap<>();

   private final Wrench residualRootJointWrench = new Wrench();
   private final FrameVector3D residualRootJointForce = new FrameVector3D();
   private final FrameVector3D residualRootJointTorque = new FrameVector3D();

   private final YoFrameVector yoResidualRootJointForce;
   private final YoFrameVector yoResidualRootJointTorque;

   private final JointIndexHandler jointIndexHandler;

   private boolean firstTick = true;

   public NewWholeBodyVirtualModelControlSolver(WholeBodyControlCoreToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      rootJoint = toolbox.getRootJoint();
      optimizationControlModule = new VirtualModelControlOptimizationControlModule(toolbox, USE_CONTACT_FORCE_QP, registry);
      centerOfMassFrame = toolbox.getCenterOfMassFrame();

      jointIndexHandler = toolbox.getJointIndexHandler();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledOneDoFJoints);
      lowLevelOneDoFJointDesiredDataHolder.setJointsControlMode(controlledOneDoFJoints, JointDesiredControlMode.EFFORT);

      controlRootBody = toolbox.getVirtualModelControlMainBody();
      virtualModelController = new NewVirtualModelController(toolbox.getJointIndexHandler());

      yoDesiredMomentumRateAngular = toolbox.getYoDesiredMomentumRateAngular();
      yoAchievedMomentumRateAngular = toolbox.getYoAchievedMomentumRateAngular();

      if (toolbox.getControlledBodies() != null)
      {
         controlledBodies = Arrays.asList(toolbox.getControlledBodies());
         wrenchVisualizer = new WrenchVisualizer("VMCDesiredExternalWrench", controlledBodies, 1.0, toolbox.getYoGraphicsListRegistry(), registry,
                                                 YoAppearance.Red(), YoAppearance.Blue());
      }
      else
      {
         controlledBodies = null;
         wrenchVisualizer = null;
      }

      if (USE_LIMITED_JOINT_TORQUES)
      {
         for (OneDoFJoint joint : controlledOneDoFJoints)
         {
            RateLimitedYoVariable jointTorqueSolution = new RateLimitedYoVariable("limited_tau_vmc_" + joint.getName(), registry, 10.0, toolbox.getControlDT());
            jointTorqueSolutions.put(joint, jointTorqueSolution);
         }
      }

      planeContactWrenchProcessor = toolbox.getPlaneContactWrenchProcessor();

      yoDesiredMomentumRateLinear = toolbox.getYoDesiredMomentumRateLinear();
      yoAchievedMomentumRateLinear = toolbox.getYoAchievedMomentumRateLinear();

      yoResidualRootJointForce = toolbox.getYoResidualRootJointForce();
      yoResidualRootJointTorque = toolbox.getYoResidualRootJointTorque();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      optimizationControlModule.initialize();
      virtualModelController.reset();
      firstTick = true;
   }

   public void clear()
   {
      optimizationControlModule.initialize();
      virtualModelController.reset();
   }

   public void initialize()
   {
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state
      // where the feet are all jacked up. For example, after falling and getting back up.
      optimizationControlModule.initialize();
      virtualModelController.reset();
      planeContactWrenchProcessor.initialize();
      firstTick = true;
   }

   public void compute()
   {
      try
      {
         optimizationControlModule.compute(virtualModelControlSolution);
      }
      catch (NewVirtualModelControlModuleException virtualModelControlModuleException)
      {
         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
         virtualModelControlSolution = virtualModelControlModuleException.getVirtualModelControlSolution();
      }

      // get output for contact forces
      Map<RigidBody, Wrench> externalWrenchSolution = virtualModelControlSolution.getExternalWrenchSolution();
      List<RigidBody> rigidBodiesWithExternalWrench = virtualModelControlSolution.getRigidBodiesWithExternalWrench();
      List<RigidBody> bodiesInContact = virtualModelControlSolution.getBodiesInContact();
      SpatialForceVector centroidalMomentumRateSolution = virtualModelControlSolution.getCentroidalMomentumRateSolution();

      yoAchievedMomentumRateLinear.set(centroidalMomentumRateSolution.getLinearPart());
      yoAchievedMomentumRateAngular.set(centroidalMomentumRateSolution.getAngularPart());
      achievedMomentumRateLinear.setIncludingFrame(yoAchievedMomentumRateLinear);

      // submit forces for contact forces
      for (int bodyIndex = 0; bodyIndex < bodiesInContact.size(); bodyIndex++)
      {
         RigidBody rigidBody = bodiesInContact.get(bodyIndex);
         externalWrenchSolution.get(rigidBody).negate();
         virtualModelController.addExternalWrench(controlRootBody, rigidBody, externalWrenchSolution.get(rigidBody),
                                                  virtualModelControlSolution.getCentroidalMomentumSelectionMatrix());
      }
      planeContactWrenchProcessor.compute(externalWrenchSolution);

      virtualModelController.populateTorqueSolution(virtualModelControlSolution);
      DenseMatrix64F jointTorquesSolution = virtualModelControlSolution.getJointTorques();

      for (RigidBody rigidBody : rigidBodiesWithExternalWrench)
      {
         externalWrenchSolution.get(rigidBody).changeBodyFrameAttachedToSameBody(rigidBody.getBodyFixedFrame());
         externalWrenchSolution.get(rigidBody).negate();
      }

      if (wrenchVisualizer != null)
         wrenchVisualizer.visualize(externalWrenchSolution);

      updateLowLevelData(jointTorquesSolution);

      rootJoint.getWrench(residualRootJointWrench);
      residualRootJointWrench.getAngularPartIncludingFrame(residualRootJointTorque);
      residualRootJointWrench.getLinearPartIncludingFrame(residualRootJointForce);
      yoResidualRootJointForce.setAndMatchFrame(residualRootJointForce);
      yoResidualRootJointTorque.setAndMatchFrame(residualRootJointTorque);
   }

   private void updateLowLevelData(DenseMatrix64F jointTorquesSolution)
   {
      rootJointDesiredConfiguration.setDesiredAccelerationFromJoint(rootJoint);

      for (OneDoFJoint joint : controlledOneDoFJoints)
      {
         int[] jointIndices = jointIndexHandler.getJointIndices(joint);

         for (int jointIndex : jointIndices)
         {
            if (USE_LIMITED_JOINT_TORQUES)
            {
               if (firstTick)
                  jointTorqueSolutions.get(joint).set(jointTorquesSolution.get(jointIndex));
               else
                  jointTorqueSolutions.get(joint).update(jointTorquesSolution.get(jointIndex));
               lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, jointTorqueSolutions.get(jointIndex).getDoubleValue());
            }
            else
            {
               lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(joint, jointTorquesSolution.get(jointIndex));
            }
         }
      }
   }

   public void submitVirtualModelControlCommandList(VirtualModelControlCommandList virtualModelControlCommandList)
   {
      while (virtualModelControlCommandList.getNumberOfCommands() > 0)
      {
         VirtualModelControlCommand<?> command = virtualModelControlCommandList.pollCommand();
         switch (command.getCommandType())
         {
         case MOMENTUM:
            optimizationControlModule.submitMomentumRateCommand((MomentumRateCommand) command);
            recordMomentumRate((MomentumRateCommand) command);
            break;
         case EXTERNAL_WRENCH:
            handleExternalWrenchCommand((ExternalWrenchCommand) command);
            break;
         case PLANE_CONTACT_STATE:
            optimizationControlModule.submitPlaneContactStateCommand((PlaneContactStateCommand) command);
            break;
         case VIRTUAL_WRENCH:
            handleVirtualWrenchCommand((VirtualWrenchCommand) command);
            break;
         case VIRTUAL_FORCE:
            handleVirtualForceCommand((VirtualForceCommand) command);
            break;
         case VIRTUAL_TORQUE:
            handleVirtualTorqueCommand((VirtualTorqueCommand) command);
            break;
         case JOINTSPACE:
            virtualModelController.addJointTorqueCommand((JointTorqueCommand) command);
            break;
         case CONTROLLED_BODIES:
            //registerAllControlledBodies((ControlledBodiesCommand) command);
            break;
         case COMMAND_LIST:
            submitVirtualModelControlCommandList((VirtualModelControlCommandList) command);
            break;
         default:
            throw new RuntimeException("The command type: " + command.getCommandType() + " is not handled by the Virtual Model Control solver mode.");
         }
      }
   }

   private void recordMomentumRate(MomentumRateCommand command)
   {
      DenseMatrix64F momentumRate = command.getMomentumRate();
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateLinear, momentumRate, 3);
      MatrixTools.extractYoFrameTupleFromEJMLVector(yoDesiredMomentumRateAngular, momentumRate, 0);
   }

   private void handleVirtualWrenchCommand(VirtualWrenchCommand commandToSubmit)
   {
      virtualModelController.addVirtualEffortCommand(commandToSubmit);

      if (commandToSubmit.getEndEffector() == controlRootBody)
      {
         commandToSubmit.getDesiredWrench(controlFrame, tempExternalWrench);
         tempExternalWrench.negate();
         optimizationControlModule.submitExternalWrench(commandToSubmit.getEndEffector(), tempExternalWrench);
      }

      commandToSubmit.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      optimizationControlModule.addSelection(tempSelectionMatrix);
   }

   private void handleVirtualForceCommand(VirtualForceCommand commandToSubmit)
   {
      virtualModelController.addVirtualEffortCommand(commandToSubmit);

      if (commandToSubmit.getEndEffector() == controlRootBody)
      {
         commandToSubmit.getDesiredWrench(controlFrame, tempExternalWrench);
         tempExternalWrench.negate();
         optimizationControlModule.submitExternalWrench(commandToSubmit.getEndEffector(), tempExternalWrench);
      }

      commandToSubmit.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      optimizationControlModule.addSelection(tempSelectionMatrix);
   }

   private void handleVirtualTorqueCommand(VirtualTorqueCommand commandToSubmit)
   {
      virtualModelController.addVirtualEffortCommand(commandToSubmit);

      if (commandToSubmit.getEndEffector() == controlRootBody)
      {
         commandToSubmit.getDesiredWrench(controlFrame, tempExternalWrench);
         tempExternalWrench.negate();
         optimizationControlModule.submitExternalWrench(commandToSubmit.getEndEffector(), tempExternalWrench);
      }

      commandToSubmit.getSelectionMatrix(centerOfMassFrame, tempSelectionMatrix);
      optimizationControlModule.addSelection(tempSelectionMatrix);
   }

   private void handleExternalWrenchCommand(ExternalWrenchCommand command)
   {
      optimizationControlModule.submitExternalWrench(command.getRigidBody(), command.getExternalWrench());
   }

   public LowLevelOneDoFJointDesiredDataHolder getOutput()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return rootJointDesiredConfiguration;
   }
}
