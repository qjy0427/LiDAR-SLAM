bag = rosbag('fifthLabScan.bag');
bSel1 = select(bag,'Topic','/odom');
bSel2 = select(bag,'Topic','/scan');
odom = readMessages(bSel1,'DataFormat','struct');
scan = readMessages(bSel2,'DataFormat','struct');

nFrames = min(bSel1.NumMessages, bSel2.NumMessages);
nScan = 512;
figure(1)
axis equal
hold on
lastFrame = zeros(3, 512);
flag = 0;

offset = 0;
% timeDiff = odom{3, 1}.Header.Stamp.Sec - scan{1, 1}.Header.Stamp.Sec;
% timeNDiff = odom{3, 1}.Header.Stamp.Nsec - scan{1, 1}.Header.Stamp.Nsec;


for iFrame = [1:50:2950, 3875:25:4650, 5000:25:nFrames] % [1:50:2950, 3875:25:4650, 5000:25:nFrames] % [1:50:2100, 2150:50:3950, 4500:25:4900, 5075:25:nFrames] % [200:50:2000, 2150:50:3500 4500:25:4900 5075:25:5300]
    
    while 1
        if odom{iFrame, 1}.Header.Stamp.Nsec - scan{iFrame + offset, 1}.Header.Stamp.Nsec > 1e8 ...
                || odom{iFrame, 1}.Header.Stamp.Sec > scan{iFrame + offset, 1}.Header.Stamp.Sec
            offset = offset + 1;
            disp(iFrame/nFrames)
        elseif odom{iFrame, 1}.Header.Stamp.Nsec - scan{iFrame + offset, 1}.Header.Stamp.Nsec < -1e8 ...
                || odom{iFrame, 1}.Header.Stamp.Sec < scan{iFrame + offset, 1}.Header.Stamp.Sec
            offset = offset - 1;
            disp(iFrame/nFrames)
        else
            break
        end
    end
    
    curFrame = zeros(3, 512);
    curPosition = [odom{iFrame,1}.Pose.Pose.Position.X; ...
        odom{iFrame,1}.Pose.Pose.Position.Y];
    curOrientation = [odom{iFrame,1}.Pose.Pose.Orientation.Z; ...
        odom{iFrame,1}.Pose.Pose.Orientation.W];
    plot(curPosition(1), curPosition(2), 'r.')
    for iScan = 1:nScan
        if scan{iFrame + offset, 1}.Ranges(iScan) == Inf || isnan(scan{iFrame + offset, 1}.Ranges(iScan))
            continue
        end
        dotX = curPosition(1) + scan{iFrame + offset, 1}.Ranges(iScan) * ...
            cos(2*acos(curOrientation(2)) + scan{1, 1}.AngleMin + iScan * scan{1, 1}.AngleIncrement);
        dotY = curPosition(2) + scan{iFrame + offset, 1}.Ranges(iScan) * ...
            sin(2*asin(curOrientation(1)) + scan{1, 1}.AngleMin + iScan * scan{1, 1}.AngleIncrement);
        curFrame(:, iScan) = [dotX; dotY; 0];
%         l = plot([curPosition(1), dotX], [curPosition(2), dotY], 'c', 'LineWidth', 1);
%         l.Color(4) = 0.25;
    end
%     if flag == 0
%         lastFrame = curFrame;
%         lastPoint = curPosition;
%     else
%         diff = curPosition - lastPoint;
%         quiver(lastPoint(1), lastPoint(2), diff(1), diff(2), 'r', 'LineWidth', 2, 'AutoScale',false)
%         lastPoint = curPosition;
%     end
%     [TR, TT] = icp(lastFrame, curFrame);
%     curFrame = (TR * curFrame + repmat(TT, 1, 512));
%     lastFrame = curFrame;
    plot(curFrame(1,:), curFrame(2,:), 'k.')
    flag = 1;
end

lastPoint = [odom{1,1}.Pose.Pose.Position.X; ...
        odom{1,1}.Pose.Pose.Position.Y];
for iFrame = 1:25:nFrames
    curPosition = [odom{iFrame,1}.Pose.Pose.Position.X; ...
        odom{iFrame,1}.Pose.Pose.Position.Y];
    diff = curPosition - lastPoint;
    quiver(lastPoint(1), lastPoint(2), diff(1), diff(2), 'r', 'LineWidth', 2, 'AutoScale',false)
    lastPoint = curPosition;
end