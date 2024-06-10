clc
close all
clear all

rng('default');
rng(11);

SAVE_RESULT = 1;
USE_LOOP_FROM = 0;
USE_RANSAC = 0;
ori_datafolder = "Dgist-RGBD_data";
ori_datafolder = "C:\hubitz\Dgist_Data\Dgist-RGBD";

for scene_num = 3:3
    if scene_num == 1
        scene_name = "Dgist-RGBD_1";
        scene_name = "scene01";
        numFiles = 1159;
        result=[100];
    elseif scene_num == 2
        scene_name = "Dgist-RGBD_2";
        scene_name = "scene02";
        numFiles = 1281;
        result=[100];
    elseif scene_num == 3
        scene_name = "Dgist-RGBD_3";
        scene_name = "scene03";
        numFiles = 1535;
        result=[100];
    else
        disp('scene num must be [1,3]');
    end

    % 범위 내의 모든 값을 생성
    result_range_left = [];
    
    for idx = 1:size(result, 1)
        temp_range = (result(idx, 1)-4):result(idx, 1);
        result_range_left = [result_range_left; temp_range.'];
    end

    result_range_right = [];
    
    for idx = 1:size(result, 1)
        temp_range = (result(idx, 1)+1):(result(idx, 1)+5);
        result_range_right = [result_range_right; temp_range.'];
    end
    

    if USE_LOOP_FROM
        LOOP_FROM = fullfile("Results", "v7", scene_name + "_loop");
    else
        LOOP_FROM = "";
    end
    
    if ~exist('Results', 'dir')
        mkdir('Results');
    end
    
    
    % 현재 시간을 얻어옴
    currentTime = datetime('now', 'Format', 'yyyyMMdd_HHmmss');
    folderName = sprintf('%s_case_%s', char(currentTime), scene_name);
    
    % 폴더 이름 생성 (Results/현재시간)
    folderName = fullfile('Results', folderName);
    
    % 폴더 생성
    if ~exist(folderName, 'dir')
        mkdir(folderName);
        fprintf('폴더 "%s"가 생성되었습니다.\n', folderName);
    else
        fprintf('폴더 "%s"는 이미 존재합니다.\n', folderName);
    end

    datapath = fullfile(ori_datafolder, scene_name);
    datas = dir(datapath);
    transfoms_txt_path = fullfile(datapath, "transform.txt");
    matdata_path = fullfile("C:\hubitz\Dgist_Data\Dgist-RGBD\save_matlab" , scene_name + "_ori.mat");
    if isfile(matdata_path)
    else
        fileID = fopen(transfoms_txt_path, 'r');
        formatSpec = "%*d: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f";
        transforms = textscan(fileID, formatSpec);
        transforms = cell2mat(transforms);
        is_global = transforms(:,17);
        transforms = transforms(:,1:16); %0925 추가, 추가로 받은 data trasnform 17번째에 쓰레기 값..
        

        fclose(fileID);
        
        numFiles = size(transforms, 1);
        scans = cell(1, 1);
        imgs = cell(1, 1);
    
        tforms = cell(numFiles, 1);
        for i = 1:numFiles
            tformmat = reshape(transforms(i, :), [4,4]);
            R = tformmat(1:3,1:3);
            t = tformmat(1:3,4);
            R = eul2rotm(rotm2eul(R));
            rtf3d = rigidtform3d(R, t);
            tforms{i} = rtf3d;
        end
        
        save_idx = 1;
        save_idx_img = 1;
        for idx=1:size(datas,1)
            filename = datas(idx).name;
            fullfilename = fullfile(datas(idx).folder, datas(idx).name);
            if ~contains(filename, ".ply")
                continue;
            end
            display(fullfilename);
            pc = pcread(fullfilename);
            total_points = pc.Count;
            scans{save_idx} = pc.Location;
            save_idx = save_idx + 1;
        end

        for idx=1:size(datas,1)
            filename = datas(idx).name;
            fullfilename = fullfile(datas(idx).folder, datas(idx).name);
            if ~contains(filename, ".png")
                continue;
            end
            display(fullfilename);
            %pc = pcread(fullfilename);
            %imageData = imread(fullfilename);
            [imageData, map, alpha] = imread(fullfilename);

            % alpha의 크기 가져오기
            [rows, cols] = size(alpha);
            
            % 새로운 크기의 배열을 만듭니다.
            RGBD = zeros(rows, cols, 4, 'uint8');
            
            % imageData와 alpha를 각각 새로운 배열에 복사합니다.
            RGBD(:, :, 1:3) = imageData;  % RGB 채널 복사
            RGBD(:, :, 4) = alpha;  % Depth 채널 복사

            imgs{save_idx_img} = RGBD;
            save_idx_img = save_idx_img + 1;
        end


        save(matdata_path, 'scans', 'tforms', 'is_global', 'imgs');
    end

    saved_mat = load(matdata_path);
    scans = saved_mat.scans(1:1:end);
    imgs = saved_mat.imgs(1:1:end);
    tforms = saved_mat.tforms;
    pcsToView = cell(1,length(scans));
    for i=1:length(scans)
        pc = scans{i};
        pcl = pointCloud(pc);
        pcToView = pcdownsample(pcl,'random', 0.05);
        pcsToView{i} = pcToView;
    end

    % 변수 설정
    step = 1; % for문의 간격
    threshold = 100; % i와 j의 차이 임계값
    SURF_threshold = 10; %낮으면 더 많이 검출
    matching_ratio = 0.9; %높으면 더 많이 검출

    % 3D 포인트 클라우드를 저장할 빈 변수 생성
   
    % % 피처 추출 및 매칭
    % all_features = cell(1, 10);
    % all_valid_points = cell(1, 10);
    
    % 피처 추출 및 매칭
    all_features = cell(1, 10);
    all_valid_points = cell(1, 10);
    numMatches=2;
    lineWidth=1;
    markerSize=10;
    
    for i = 690:step:length(scans)
        % 피처 추출
        for k = 1:4
            rgbd = imgs{i+k};  % 여기서 k를 i+k로 변경
            rgb = rgbd(:, :, 1:3);
            grayImage = rgb2gray(rgb);
            points = detectSURFFeatures(grayImage);
            [features, valid_points] = extractFeatures(grayImage, points);
            all_features{k} = features;
            all_valid_points{k} = valid_points;
        end
    
        % 이미지 배치 설정
        numImages = 4;
        imageHeight = 480;
        imageWidth = 400;
        rows = 2;
        cols = 2;
        gap = 50;  % 이미지 사이의 공간
    
        combinedHeight = rows * imageHeight + (rows - 1) * gap;
        combinedWidth = cols * imageWidth + (cols - 1) * gap;
        combinedImage = zeros(combinedHeight, combinedWidth, 3, 'uint8');
    
        for k = 1:numImages
            row = floor((k-1) / cols);
            col = mod(k-1, cols);
            startY = row * (imageHeight + gap) + 1;
            startX = col * (imageWidth + gap) + 1;
            combinedImage(startY:startY + imageHeight - 1, startX:startX + imageWidth - 1, :) = imgs{i+k}(:, :, 1:3);
        end
        
        hold off;
        % figure;
        imshow(combinedImage);
        hold on;
    
        % 충분한 색상 팔레트를 생성합니다.
        numColors = numImages * numMatches;
        colors = hsv(numColors);  % numColors만큼 고유한 색상을 생성
    
        colorIndex = 1;
        matchColors = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
        % 각 이미지 간의 매칭 포인트를 저장할 맵
        uniqueMatchesMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
        for m = 1:numImages
            for n = 1:numImages
                if m ~= n
                    features1 = all_features{m};
                    valid_points1 = all_valid_points{m};
                    features2 = all_features{n};
                    valid_points2 = all_valid_points{n};
    
                    % 매칭
                    indexPairs = matchFeatures(features1, features2);
                    matchedPoints1 = valid_points1(indexPairs(:, 1), :);
                    matchedPoints2 = valid_points2(indexPairs(:, 2), :);
    
                    % 매칭된 포인트 시각화
                    loc1 = matchedPoints1.Location + [(mod(m-1, cols)) * (imageWidth + gap), floor((m-1) / cols) * (imageHeight + gap)];  % x, y 좌표를 이미지 간 간격으로 조정
                    loc2 = matchedPoints2.Location + [(mod(n-1, cols)) * (imageWidth + gap), floor((n-1) / cols) * (imageHeight + gap)];  % x, y 좌표를 이미지 간 간격으로 조정
    
                    % 매칭된 포인트 랜덤 선택
                    numTotalMatches = size(loc1, 1);
                    if numTotalMatches > numMatches
                        randIndices = randperm(numTotalMatches, numMatches);
                        loc1 = loc1(randIndices, :);
                        loc2 = loc2(randIndices, :);
                    end
    
                    % 매칭된 포인트 연결선 그리기 및 마킹
                    for p = 1:size(loc1, 1)
                        key1 = sprintf('%d_%d', loc1(p, 1), loc1(p, 2));
                        key2 = sprintf('%d_%d', loc2(p, 1), loc2(p, 2));
                        if isKey(matchColors, key1)
                            color = matchColors(key1);
                        elseif isKey(matchColors, key2)
                            color = matchColors(key2);
                        else
                            color = colors(mod(colorIndex-1, numColors) + 1, :);
                            matchColors(key1) = color;
                            matchColors(key2) = color;
                            colorIndex = colorIndex + 1;
                        end
    
                        plot([loc1(p, 1), loc2(p, 1)], [loc1(p, 2), loc2(p, 2)], 'Color', color, 'LineWidth', lineWidth);
                        plot(loc1(p, 1), loc1(p, 2), 'x', 'Color', color, 'MarkerSize', markerSize, 'LineWidth', lineWidth);
                        plot(loc2(p, 1), loc2(p, 2), 'x', 'Color', color, 'MarkerSize', markerSize, 'LineWidth', lineWidth);
                    end
                end
            end
        end
    
        title('Matched Points across 4 Images');
        ttt=1;
    end


    for i = 1:step:length(scans)
         combined_pc = pointCloud([0, 0, 0]);
    end




    % for i = 1:step:length(scans)
    %     for j = 1:step:length(scans)
    %         % i와 j의 차이가 threshold 이하인 경우 건너뛰기
    %         if abs(i - j) <= threshold
    %             continue;
    %         end
    % 
    %         source_img = imgs{i};
    %         target_img = imgs{j};
    % 
    %         % 특징점 추출을 위해 이미지를 회색조로 변환
    %         graySource = rgb2gray(source_img);
    %         grayTarget = rgb2gray(target_img);
    % 
    %         % 이미지 전처리 (대비 조정)
    %         graySource = imadjust(graySource);
    %         grayTarget = imadjust(grayTarget);
    % 
    %         % SIFT 특징점 추출
    %         pointsSource = detectSURFFeatures(graySource, 'MetricThreshold', SURF_threshold, 'NumOctaves', 3, 'NumScaleLevels', 4);
    %         pointsTarget = detectSURFFeatures(grayTarget, 'MetricThreshold', SURF_threshold, 'NumOctaves', 3, 'NumScaleLevels', 4);
    % 
    %         % 특징점이 충분한지 확인
    %         if pointsSource.Count < 2 || pointsTarget.Count < 2
    %             disp('Not enough keypoints detected in one or both images.');
    %             continue;
    %         end
    % 
    %         [featuresSource, validPointsSource] = extractFeatures(graySource, pointsSource);
    %         [featuresTarget, validPointsTarget] = extractFeatures(grayTarget, pointsTarget);
    % 
    %         % 특징점 매칭
    %         indexPairs = matchFeatures(featuresSource, featuresTarget, 'MaxRatio', matching_ratio);
    % 
    %         % 중복된 타겟 특징점 제거
    %         [uniqueTargetIdx, uniqueIdx] = unique(indexPairs(:, 2), 'stable');
    %         indexPairs = indexPairs(uniqueIdx, :);
    % 
    %         matchedPointsSource = validPointsSource(indexPairs(:, 1));
    %         matchedPointsTarget = validPointsTarget(indexPairs(:, 2));
    % 
    %         % 매칭 점의 수가 충분한지 확인
    %         if size(matchedPointsSource, 1) < 4 || size(matchedPointsTarget, 1) < 4
    %             disp('Not enough matched points to estimate geometric transform.');
    %             continue;
    %         end
    % 
    %         try
    %             % RANSAC을 사용하여 기하적 변환 모델 추정
    %             [tform, inlierPointsSource, inlierPointsTarget] = estimateGeometricTransform(matchedPointsSource, matchedPointsTarget, 'similarity');
    % 
    %             % Loop closure 확인
    %             if size(inlierPointsSource, 1) > 10 % 임계값, inlier 매칭 점의 수가 충분히 많으면 Loop closure로 간주
    %                 disp(['Loop closure detected between frames ' num2str(i) ' and ' num2str(j)]);
    % 
    %                 % 포인트 클라우드 불러오기
    %                 pcSource = pcsToView{i};
    %                 pcTarget = pcsToView{j};
    % 
    %                 % 포인트 클라우드 좌표계 변환
    %                 pcSource_trans = pctransform(pcSource, tforms{i});
    %                 pcTarget_trans = pctransform(pcTarget, tforms{j});
    % 
    %                 % 포인트 클라우드 시각화
    %                 figure;
    %                 pcshow(pcSource_trans.Location, 'r', 'MarkerSize', 50); % 빨간색으로 시각화
    %                 hold on;
    %                 pcshow(pcTarget_trans.Location, 'b', 'MarkerSize', 50); % 파란색으로 시각화
    %                 title('Transformed Point Clouds with Loop Closure');
    %                 hold off;
    % 
    %                 figure;
    %                 showMatchedFeatures(source_img, target_img, inlierPointsSource, inlierPointsTarget, 'montage');
    %                 title('Matched Points (Inliers Only)');
    %                 legend('matched points 1','matched points 2');
    %                 tt = 1; % 이 줄은 코드 흐름을 멈추지 않습니다.
    % 
    %             else
    %                 disp(['Loop closure not detected between frames ' num2str(i) ' and ' num2str(j)]);
    % 
    %             end
    %         catch ME
    %             disp('Error estimating geometric transform.');
    %             disp(ME.message);
    %             continue;
    %         end
    %     end
    % end

end


function removed_pc = removeRandomPoints(pc, remove_percent)
    % pointCloud에서 점의 총 개수 얻기
    total_points = pc.Count;
    % 제거할 점의 개수 (n = 제거할 비율)
    remove_points = round(remove_percent * total_points);
    % 전체 점들 중에서 무작위로 제거할 점들의 인덱스를 선택
    remove_indices = randperm(total_points, remove_points);
    % 제거할 점들을 빼고 새로운 pointCloud를 만들기
    removed_pc = select(pc, setdiff(1:total_points, remove_indices));
end


function [R_norm, T_norm] = computeNorms(tform)
    % 회전 행렬 추출
    R = tform.T(1:3, 1:3);
    
    % 회전 행렬 R을 Euler 각으로 변환
    yaw = atan2(R(2,1), R(1,1));
    pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    roll = atan2(R(3,2), R(3,3));
    
    % 라디안에서도 출력되는 Euler 각을 도로 변환
    yaw = rad2deg(yaw);
    pitch = rad2deg(pitch);
    roll = rad2deg(roll);
    
    % Euler 각을 벡터로 묶음
    ypr = [yaw, pitch, roll];
    
    % R_norm과 T_norm 계산
    R_norm = norm(ypr);
    T_norm = norm(tform.Translation);
end

function FPFH_features = computeFPFHFeatures(newPointCloud, radius)
    % 초기화
    numPoints = newPointCloud.Count;
    FPFH_features = zeros(numPoints, 33);

    % 모든 포인트에 대해서
    for i = 1:numPoints
        % 중심 포인트의 위치와 노멀
        p_center = newPointCloud.Location(i, :);
        n_center = newPointCloud.Normal(i, :);
        intensity_center = newPointCloud.Intensity(i);

        % 이웃 찾기
        indices = findNeighborsInRadius(newPointCloud, p_center, radius);

        % Intensity 기준으로 추가 필터링
        intensities = newPointCloud.Intensity(indices);
        valid_intensity_idx = indices(abs(intensities - intensity_center) < 100);

        % 히스토그램 초기화
        hist_SPF1 = zeros(1, 11);
        hist_SPF2 = zeros(1, 11);
        hist_SPF3 = zeros(1, 11);

        % 각 이웃에 대해서
        for j = 1:length(valid_intensity_idx)
            idx = valid_intensity_idx(j);
            if idx == i  % 자기 자신은 제외
                continue;
            end

            % 이웃 포인트의 위치와 노멀
            p_neighbor = newPointCloud.Location(idx, :);
            n_neighbor = newPointCloud.Normal(idx, :);

            % 두 포인트 사이의 벡터
            d = p_neighbor - p_center;

            % 각도 계산
            angle1 = acos(dot(n_center, d) / (norm(n_center) * norm(d))) / pi;
            angle2 = acos(dot(n_neighbor, d) / (norm(n_neighbor) * norm(d))) / pi;
            angle3 = acos(dot(n_center, n_neighbor) / (norm(n_center) * norm(n_neighbor))) / pi;

            % SPF 히스토그램 업데이트
            bin1 = floor(angle1 * 10) + 1;
            bin2 = floor(angle2 * 10) + 1;
            bin3 = floor(angle3 * 10) + 1;

            hist_SPF1(bin1) = hist_SPF1(bin1) + 1;
            hist_SPF2(bin2) = hist_SPF2(bin2) + 1;
            hist_SPF3(bin3) = hist_SPF3(bin3) + 1;
        end
        hist_SPF1 = hist_SPF1 / sum(hist_SPF1);
        hist_SPF2 = hist_SPF2 / sum(hist_SPF2);
        hist_SPF3 = hist_SPF3 / sum(hist_SPF3);

        % FPFH 특징 계산
        FPFH_features(i, :) = [hist_SPF1, hist_SPF2, hist_SPF3];
    end
end


function outpc = myPcdownsample(inpc, grid_size)
    loc = inpc.Location;
    intensity = inpc.Intensity;
    min_loc = min(loc);
    max_loc = max(loc);

    
    % 그리드 위치 계산
    grid_loc = floor((loc - min_loc) / grid_size) + 1;

    % 유일한 그리드 위치와 각 위치의 인덱스 계산
    [unique_grid_loc, ~, idx] = unique(grid_loc, 'rows', 'stable');

    numUnique = size(unique_grid_loc, 1);
    avg_loc = zeros(0, 3);  % 0x3 크기의 배열로 초기화
    random_intensity = [];

    for i = 1:numUnique
        mask = (idx == i);

        if any(mask)  % mask가 전부 false가 아닌 경우만 계산
            avg_loc = vertcat(avg_loc, mean(loc(mask, :), 1));
            % 해당 복셀 내의 intensity 값 중에서 랜덤하게 하나 선택
            possible_intensities = intensity(mask);
            random_intensity = vertcat(random_intensity, possible_intensities(randi(length(possible_intensities))));
        end
    end

    % 새로운 pointCloud 객체 생성
    outpc = pointCloud(avg_loc, 'Intensity', random_intensity);
end

function mkdir_if_not_exist(folder)
    if ~exist(folder, 'dir')
        mkdir(folder);
    end
end