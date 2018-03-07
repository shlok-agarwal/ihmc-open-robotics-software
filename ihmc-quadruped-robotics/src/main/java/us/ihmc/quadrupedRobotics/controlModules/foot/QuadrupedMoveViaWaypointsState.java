package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedMoveViaWaypointsState extends QuadrupedUnconstrainedFootState
{
   // Yo variables
   private final YoDouble robotTime;

   // SoleWaypoint variables
   private final MultipleWaypointsPositionTrajectoryGenerator quadrupedWaypointsPositionTrajectoryGenerator;

   // Feedback controller
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame soleFrame;
   private final QuadrupedSoleWaypointList quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();

   private final QuadrupedFootControlModuleParameters parameters;
   private final RobotQuadrant robotQuadrant;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private double taskStartTime;

   public QuadrupedMoveViaWaypointsState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox,
                                         QuadrupedSolePositionController solePositionController, YoVariableRegistry registry)
   {
      super("moveViaWaypoints", robotQuadrant, controllerToolbox, solePositionController, registry);

      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;
      this.bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();
      this.soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
      this.parameters = controllerToolbox.getFootControlModuleParameters();
      robotTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      // Create waypoint trajectory
      quadrupedWaypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(robotQuadrant.getCamelCaseName() + "SoleTrajectory",
                                                                                                       bodyFrame, registry);
   }

   public void handleWaypointList(QuadrupedSoleWaypointList quadrupedSoleWaypointList)
   {
      this.quadrupedSoleWaypointList.set(quadrupedSoleWaypointList);
   }

   public void initialize(boolean useInitialSoleForceAsFeedforwardTerm)
   {
      solePositionControllerSetpoints.initialize(soleFrame);
      solePositionController.reset();

      if (useInitialSoleForceAsFeedforwardTerm)
      {
         this.initialSoleForces.setIncludingFrame(controllerToolbox.getTaskSpaceEstimates().getSoleVirtualForce(robotQuadrant));
         this.initialSoleForces.changeFrame(bodyFrame);
      }
      else
      {
         this.initialSoleForces.setToZero(bodyFrame);
      }

      createSoleWaypointTrajectory();
      taskStartTime = robotTime.getDoubleValue();
   }

   @Override
   public void onEntry()
   {
      solePositionController.reset();
      solePositionController.getGains().set(parameters.getSolePositionGains());
      solePositionControllerSetpoints.initialize(soleFrame);
   }

   @Override
   public QuadrupedFootControlModule.FootEvent process()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > quadrupedSoleWaypointList.getFinalTime())
      {
         soleForceCommand.setToZero();

         if (waypointCallback != null)
            waypointCallback.isDoneMoving(true);

         solePositionControllerSetpoints.getSolePosition().set(controllerToolbox.getTaskSpaceEstimates().getSolePosition(robotQuadrant));
         solePositionControllerSetpoints.getSoleLinearVelocity().setToZero();
         super.doControl();

         return QuadrupedFootControlModule.FootEvent.TIMEOUT;
      }
      else
      {
         quadrupedWaypointsPositionTrajectoryGenerator.compute(currentTrajectoryTime);
         quadrupedWaypointsPositionTrajectoryGenerator.getPosition(solePositionControllerSetpoints.getSolePosition());
         solePositionControllerSetpoints.getSoleLinearVelocity().setToZero();
         solePositionControllerSetpoints.getSoleForceFeedforward().setIncludingFrame(initialSoleForces);
         solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, controllerToolbox.getTaskSpaceEstimates().getSoleLinearVelocity(robotQuadrant));

         super.doControl();

         if (waypointCallback != null)
            waypointCallback.isDoneMoving(false);

         return null;
      }
   }

   @Override
   public void onExit()
   {
      soleForceCommand.setToZero();
   }

   private void createSoleWaypointTrajectory()
   {
      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      for (int i = 0; i < quadrupedSoleWaypointList.size(); ++i)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.appendWaypoint(quadrupedSoleWaypointList.get(i).getTime(),
                                                                      quadrupedSoleWaypointList.get(i).getPosition(),
                                                                      quadrupedSoleWaypointList.get(i).getVelocity());
      }
      if (quadrupedSoleWaypointList.size() > 0)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.initialize();
      }
   }
}
