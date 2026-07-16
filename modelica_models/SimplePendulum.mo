model SimplePendulum
  parameter Real L = 1.0;
  parameter Real g = 9.81;
  Real theta(start = 0.5);
equation
  der(der(theta)) = -(g / L) * sin(theta);
  annotation(experiment(StopTime = 10.0));
end SimplePendulum;
