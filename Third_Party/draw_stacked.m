function [ ok ] = draw_stacked( time_vector)

close all
paint   = time_vector(:,1);
pre_dec = time_vector(:,3);
dec     = time_vector(:,2);


bar([paint, pre_dec, dec], 'stacked')
title('Center Mapping');


xlabel('Time in Sequence') % x-axis label
ylabel('Processing Time') % y-axis label
legend('Paint','Pre-Decomposition', 'Decomposition','Location','northeast')
axis([0 Inf 0 150])
