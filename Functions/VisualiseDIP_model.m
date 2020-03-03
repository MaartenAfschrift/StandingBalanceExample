function [] = VisualiseDIP_model(R,P,tend)
%UNTITLED Summary of this function goes here
%   R: structure with results
%   P: model properties
%   tend: final time visualisation

% Author: Maarten Afschrift

% get the hip and truk location
l1  = 0.5*P.l;
l2  = 0.5.*P.l;
th1 = R.q(:,1) + pi/2;
th2 = R.q(:,2) + th1;
i   = [1 0 0];
j   = [0 1 0];
e1  = cos(th1)*i + sin(th1)*j;  %Unit vector along first link
e2  = cos(th2)*i + sin(th2)*j;  %Unit vector along first link
p1  = l1*e1;     % Position of the first joint
p2  = p1 + l2*e2;            % Position of the second joint
loc_hipy=p1(:,2);
loc_hipx=p1(:,1);
loc_heady=p2(:,2);
loc_headx=p2(:,1);

iend = find(R.t>tend,1,'first');

figure()
for i=1:length(R.t(1:iend))  
    % plot joints
    plot(loc_hipx(i),loc_hipy(i),'ok','MarkerFaceColor',[0 0 0],'MarkerSize',6);hold on
    plot(loc_headx(i),loc_heady(i),'ok','MarkerFaceColor',[0 0 0],'MarkerSize',6);hold on
    plot(0,0,'ok','MarkerFaceColor',[0 0 0],'MarkerSize',6);
    line([loc_hipx(i) loc_headx(i)],[loc_hipy(i) loc_heady(i)])
    line([loc_hipx(i) 0],[loc_hipy(i) 0])
    % plot foot
    line([0 0.2],[0.1 0]);
    line([0 -0.05],[0.1 0]);  
    line([-0.05 0.2],[0 0]);  
    % limit axis 
    set(gca,'XLim',[-2 2]);
    set(gca,'YLim',[-1 2]);
    pause(0.001);
    if i== 1
       disp('Hit enter to start visualisation');
       pause; 
    end    
    hold off;
end








end

