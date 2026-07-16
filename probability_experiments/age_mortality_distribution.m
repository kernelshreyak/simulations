% MORTALITY_MODEL_WITH_CHILD_MORTALITY
% This script:
% 1) Combines child mortality from a piecewise PDF with a quadratic PDF for older ages.
% 2) Normalizes the combined PDF.
% 3) Plots the combined PDF and conditional probabilities.


clc;
close all;

%--- Define the unnormalized combined PDF ---
combined_pdf = @(t) ( ...
    (0.3007 * exp(-t) .* (t >= 0 & t < 5)) + ...       % Child mortality component
    (3e-9 * t.^2 .* (100 - t).^2 .* (t >= 5 & t <= 100)) ... % Quadratic component
);

%--- Compute the normalization constant ---
total_integral = quad(combined_pdf, 0, 100);
normalization_constant = 1 / total_integral;

%--- Define the normalized combined PDF ---
mortality_pdf = @(t) normalization_constant * combined_pdf(t);

%--- Verify normalization ---
normalized_integral = quad(mortality_pdf, 0, 100);
disp(["Check total PDF integral (should be 1): ", num2str(normalized_integral)]);

%--- Plot the Combined PDF ---
ages = 0:0.1:100;  % Age range for plotting
pdf_values = arrayfun(mortality_pdf, ages);

figure;
plot(ages, pdf_values, 'LineWidth', 2);
title('Combined Mortality PDF (Child Mortality + Quadratic)');
xlabel('Age');
ylabel('Probability Density');
grid on;

%--- Calculate Conditional Probabilities ---
% Conditional probability: P(t <= age < t+10 | age >= t)
interval = 10;  % Interval width
age_start = 0:10:90;  % Start of each interval
conditional_probs = zeros(size(age_start));  % Placeholder for results

for i = 1:length(age_start)
    t = age_start(i);
    num = quad(mortality_pdf, t, t + interval);     % Numerator: P(t <= age < t+10)
    den = quad(mortality_pdf, t, 100);              % Denominator: P(age >= t)
    conditional_probs(i) = num / den;               % Conditional probability
end

%--- Plot Conditional Probabilities ---
figure;
plot(age_start, conditional_probs, '-o', 'LineWidth', 2, 'MarkerSize', 8);
title('Conditional Probability of Death in the Next 10 Years (Combined PDF)');
xlabel('Starting Age (t)');
ylabel('P(t <= age < t+10 | age >= t)');
grid on;

%--- Display Results ---
disp("Conditional probabilities of dying within the next 10 years given survival to age t:");
for i = 1:length(age_start)
    fprintf("  Age %2d to %3d: %.4f\n", age_start(i), age_start(i) + interval, conditional_probs(i));
end

