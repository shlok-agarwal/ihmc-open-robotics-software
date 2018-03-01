package us.ihmc.exampleSimulations.forceSensorExamples;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class FallingBoxRobotDescription extends RobotDescription
{
   private Box3D rootBox;
   private Box3D bodyBox;

   public FallingBoxRobotDescription(String name, Box3D rootBox, Box3D bodyBox, double mass, boolean useGroundContactPoints)
   {
      super(name);

      this.rootBox = rootBox;
      this.bodyBox = bodyBox;

      double bodyBoxHeight = bodyBox.getHeight();
      double bodyBoxLength = bodyBox.getLength();
      double bodyBoxWidth = bodyBox.getWidth();

      FloatingJointDescription rootJoint = new FloatingJointDescription("rootJoint", "bodyjointvariablename");

      LinkDescription rootLink = new LinkDescription("rootLink");
      rootLink.setMassAndRadiiOfGyration(1.0, rootBox.getLength(), rootBox.getWidth(), rootBox.getHeight());

      LinkGraphicsDescription rootLinkGraphics = new LinkGraphicsDescription();
      rootLinkGraphics.translate(0.0, 0.0, -0.5 * rootBox.getHeight());
      rootLinkGraphics.addCube(rootBox.getLength(), rootBox.getWidth(), rootBox.getHeight(), YoAppearance.Blue());
      rootLinkGraphics.identity();
      rootLink.setLinkGraphics(rootLinkGraphics);

      rootJoint.setLink(rootLink);

      addRootJoint(rootJoint);

      PinJointDescription bodyJoint = new PinJointDescription("bodyJoint", new Vector3D(0.0, 0.0, -0.5 * rootBox.getHeight()), Axis.Z);

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, -0.5 * bodyBoxHeight));
      bodyLink.setMassAndRadiiOfGyration(mass, bodyBoxLength, bodyBoxWidth, bodyBoxHeight);

      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.translate(0.0, 0.0, -bodyBoxHeight);
      bodyLinkGraphics.addCube(bodyBoxLength, bodyBoxWidth, bodyBoxHeight, YoAppearance.Red());
      bodyLinkGraphics.identity();

      bodyLink.setLinkGraphics(bodyLinkGraphics);

      bodyJoint.setLink(bodyLink);

      if (useGroundContactPoints)
      {
         int idOfGCP = 0;
         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, 0.5 * bodyBoxLength, 0.5 * bodyBoxWidth, 0.5 * bodyBoxHeight - 0.5 * bodyBoxHeight);
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, 0.5 * bodyBoxLength, -0.5 * bodyBoxWidth, 0.5 * bodyBoxHeight - 0.5 * bodyBoxHeight);
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, 0.5 * bodyBoxLength, 0.5 * bodyBoxWidth, -0.5 * bodyBoxHeight - 0.5 * bodyBoxHeight);
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, 0.5 * bodyBoxLength, -0.5 * bodyBoxWidth, -0.5 * bodyBoxHeight - 0.5 * bodyBoxHeight);
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, -0.5 * bodyBoxLength, 0.5 * bodyBoxWidth, 0.5 * bodyBoxHeight - 0.5 * bodyBoxHeight);
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, -0.5 * bodyBoxLength, -0.5 * bodyBoxWidth, 0.5 * bodyBoxHeight - 0.5 * bodyBoxHeight);
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, -0.5 * bodyBoxLength, 0.5 * bodyBoxWidth, -0.5 * bodyBoxHeight - 0.5 * bodyBoxHeight);
         idOfGCP++;

         addGroundContactPoint(bodyJoint, name + "_gc" + idOfGCP, -0.5 * bodyBoxLength, -0.5 * bodyBoxWidth, -0.5 * bodyBoxHeight - 0.5 * bodyBoxHeight);
      }

      RigidBodyTransform sensorLocation = new RigidBodyTransform();
      ForceSensorDescription ftSensor = new ForceSensorDescription(name + "_ft", sensorLocation);
      ftSensor.setUseGroundContactPoints(useGroundContactPoints);
      bodyJoint.addForceSensor(ftSensor);

      rootJoint.addJoint(bodyJoint);
   }

   private void addGroundContactPoint(JointDescription joint, String name, double x, double y, double z)
   {
      GroundContactPointDescription gcpDescription = new GroundContactPointDescription(name, new Vector3D(x, y, z));
      joint.addGroundContactPoint(gcpDescription);

      joint.getLink().getLinkGraphics().identity();
      joint.getLink().getLinkGraphics().translate(x, y, z);
      joint.getLink().getLinkGraphics().addSphere(0.01);
   }

   public Box3D getBodyBox()
   {
      return bodyBox;
   }

   public Box3D getRootBox()
   {
      return rootBox;
   }

}
