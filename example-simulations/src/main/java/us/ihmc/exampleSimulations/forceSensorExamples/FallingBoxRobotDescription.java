package us.ihmc.exampleSimulations.forceSensorExamples;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class FallingBoxRobotDescription extends RobotDescription
{
   public FallingBoxRobotDescription(String name, Box3D box, double mass, boolean useGroundContactPoint)
   {
      super(name);

      FloatingJointDescription bodyJoint = new FloatingJointDescription("body", "bodyjointvariablename");

      // TODO : add two force sensors on body.
      //      ForceSensorDescription forceSensorDescriptionOne = new ForceSensorDescription("ftsensorone", new RigidBodyTransform());
      //      forceSensorDescriptionOne.setUseGroundContactPoints(false);

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setMassAndRadiiOfGyration(mass, box.getLength(), box.getWidth(), box.getHeight());

      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.translate(0.0, 0.0, -0.5 * box.getHeight());
      bodyLinkGraphics.addCube(box.getLength(), box.getWidth(), box.getHeight(), YoAppearance.Red());
      bodyLinkGraphics.identity();
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      bodyJoint.setLink(bodyLink);

      if (useGroundContactPoint)
      {
         int idOfGCP = 0;
         GroundContactPointDescription gcpDescription1 = new GroundContactPointDescription("gc"
               + idOfGCP, new Vector3D(0.5 * box.getLength(), 0.5 * box.getWidth(), 0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription1);
         idOfGCP++;

         GroundContactPointDescription gcpDescription2 = new GroundContactPointDescription("gc"
               + idOfGCP, new Vector3D(0.5 * box.getLength(), 0.5 * box.getWidth(), -0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription2);
         idOfGCP++;

         GroundContactPointDescription gcpDescription3 = new GroundContactPointDescription("gc"
               + idOfGCP, new Vector3D(0.5 * box.getLength(), -0.5 * box.getWidth(), 0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription3);
         idOfGCP++;

         GroundContactPointDescription gcpDescription4 = new GroundContactPointDescription("gc"
               + idOfGCP, new Vector3D(0.5 * box.getLength(), -0.5 * box.getWidth(), -0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription4);
         idOfGCP++;

         GroundContactPointDescription gcpDescription5 = new GroundContactPointDescription("gc"
               + idOfGCP, new Vector3D(-0.5 * box.getLength(), 0.5 * box.getWidth(), 0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription5);
         idOfGCP++;

         GroundContactPointDescription gcpDescription6 = new GroundContactPointDescription("gc"
               + idOfGCP, new Vector3D(-0.5 * box.getLength(), 0.5 * box.getWidth(), -0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription6);
         idOfGCP++;

         GroundContactPointDescription gcpDescription7 = new GroundContactPointDescription("gc"
               + idOfGCP, new Vector3D(-0.5 * box.getLength(), -0.5 * box.getWidth(), 0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription7);
         idOfGCP++;

         GroundContactPointDescription gcpDescription8 = new GroundContactPointDescription("gc"
               + idOfGCP, new Vector3D(-0.5 * box.getLength(), -0.5 * box.getWidth(), -0.5 * box.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription8);
         idOfGCP++;
      }

      this.addRootJoint(bodyJoint);
   }

}
