model ThreeLinkRobot
  import MB = Modelica.Mechanics.MultiBody;
  inner MB.World world(gravityType = MB.Types.GravityTypes.UniformGravity);
  MB.Parts.Fixed base;
  MB.Joints.Revolute joint1(phi(start = 0.5, fixed = true), w(start = 0));
  MB.Joints.Revolute joint2(phi(start = -0.3, fixed = true), w(start = 0));
  MB.Joints.Revolute joint3(phi(start = 0.2, fixed = true), w(start = 0));
  MB.Parts.BodyCylinder link1(length = 1.0, diameter = 0.08, density = 7800);
  MB.Parts.BodyCylinder link2(length = 0.8, diameter = 0.07, density = 7800);
  MB.Parts.BodyCylinder link3(length = 0.6, diameter = 0.06, density = 7800);
equation
  connect(base.frame_b, joint1.frame_a);
  connect(joint1.frame_b, link1.frame_a);
  connect(link1.frame_b, joint2.frame_a);
  connect(joint2.frame_b, link2.frame_a);
  connect(link2.frame_b, joint3.frame_a);
  connect(joint3.frame_b, link3.frame_a);
  annotation(experiment(StopTime = 10.0));
end ThreeLinkRobot;
