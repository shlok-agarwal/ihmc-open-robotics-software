package us.ihmc.robotModels;

import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

import java.util.List;

public interface FullQuadrupedRobotModel extends FullLeggedRobotModel<RobotQuadrant>
{
   JointLimit getJointLimit(QuadrupedJointName jointName);

   QuadrupedJointName getNameForOneDoFJoint(OneDoFJoint oneDoFJoint);

   List<OneDoFJoint> getLegJointsList(RobotQuadrant robotQuadrant);

   /**
    * Returns the {@link RigidBody} describing the body of this robot.
    * In the current framework (on the day: 3/1/2014), the body is the the first successor of the root joint.
    */
   RigidBody getBody();

   default RobotQuadrant[] getRobotSegments()
   {
      return RobotQuadrant.values;
   }
}