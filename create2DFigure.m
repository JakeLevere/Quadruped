%% Figure Function
% This funciton creates a figure with the given parameters. This does not
% include a legend

function [fig] = create2DFigure(name, xPoints, yPoints, lineColor, limits, labels, Title)
fig = figure('Name', name);
plot(xPoints,yPoints,lineColor);
hold on
xlim([limits(1,1),limits(1,2)]);
ylim([limits(2,1),limits(2,2)]);
xlabel(labels(1))
ylabel(labels(2))
title(Title)
end