bag = rosbag('hallWay2.bag');
% bSel1 = select(bag,'Topic','/odom');
bSel2 = select(bag,'Topic','/scan');
% odom = readMessages(bSel1,'DataFormat','struct');
scan = readMessages(bSel2,'DataFormat','struct');

for iScan = 1:length(scan)
    scan{iScan, 1} = lidarScan(double(scan{iScan, 1}.Ranges), double(1.5464:-0.0061:-1.5708));
end

maxRange = 5.6;
resolution = 20;
slamObj = lidarSLAM(resolution,maxRange);
slamObj.LoopClosureThreshold = 360;
slamObj.LoopClosureSearchRadius = 8;

for iFrame = 1001:10:3001
    if mod(iFrame, 10) == 1
        disp(iFrame)
    end
    addScan(slamObj,scan{iFrame, 1});
end
show(slamObj)
[scansSLAM,poses] = scansAndPoses(slamObj);
occMap = buildMap(scansSLAM,poses,resolution,maxRange);
figure
show(occMap)
title('Occupancy Map of Lab')