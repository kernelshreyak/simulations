model Pendulum3D
  inner Modelica.Mechanics.MultiBody.World world;
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed;
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute(phi(start = 0.5));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder rod(length = 1, diameter = 0.05, density = 7800);
equation
  connect(fixed.frame_b, revolute.frame_a);
  connect(revolute.frame_b, rod.frame_a);
  annotation(experiment(StopTime = 10.0));
end Pendulum3D;
