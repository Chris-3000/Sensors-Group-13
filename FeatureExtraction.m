classdef FeatureExtraction
    methods(Static)    
        function plotFeatures(filename, technique)
            %% Read in image
            image = imread(filename);

            %% Make a grayscale copy
            imageGray = rgb2gray(image);
            
            %% Find features of the image
            if strcmp(technique, 'Harris')
                features = detectHarrisFeatures(imageGray,'MinQuality',0.3,'FilterSize',7);
            elseif strcmp(technique, 'ORB')
                features = detectORBFeatures(imageGray, 'NumLevels',3);
            else
                disp("Error: Function argument only accepts 'Harris' or 'ORB'.");
                return
            end
            disp(features.Count);
            
            %% Display the image and its features
            imshow(image);
            hold on;
            % disp(features.Location)
            scatter(features.Location(:,1), features.Location(:,2), 'r*');
            labels = dbscan(features.Location, 45, 10);

            uniqueLabels = unique(labels);
            uniqueLabels(uniqueLabels == -1) = [];
            meanPoints = zeros(length(uniqueLabels), 2);
            for i = 1:length(uniqueLabels)
                % Get the points in the current cluster
                clusterPoints = features.Location(labels == uniqueLabels(i), :);
                
                % Compute the mean of the cluster points
                meanPoints(i, :) = mean(clusterPoints, 1);
            end
            % scatter(meanPoints(:,1), meanPoints(:,2), 'go');

            % currentPoint = 1;
            % for i = 1:features.Count
            %     if labels(i) == currentPoint
            %         if currentPoint > 3 && currentPoint <= 3+28
            %             scatter(features.Location(i,1), features.Location(i,2), 'go');
            %         end
            %         currentPoint = currentPoint + 1;
            %     end
            %     % if currentPoint > 31
            %     %     break;
            %     % end
            % end
            for i = 1:length(uniqueLabels)
                if i == 17
                    scatter(meanPoints(i,1), meanPoints(i,2), 'go')
                end
            end
            % disp(currentPoint - 1);
            
        end

        function detectingAndMatching(technique)
            %% Read in images
            roofs1 = imread("roofs1(1).jpg");
            roofs2 = imread("roofs2(1).jpg");

            %% Make grayscale copies
            roofs1Gray = rgb2gray(roofs1);
            roofs2Gray = rgb2gray(roofs2);

            %% Extract features of the images
            if strcmp(technique, 'Harris')
                roofs1Features = detectHarrisFeatures(roofs1Gray);
                roofs2Features = detectHarrisFeatures(roofs2Gray);
            elseif strcmp(technique, 'ORB')
                roofs1Features = detectORBFeatures(roofs1Gray);
                roofs2Features = detectORBFeatures(roofs2Gray);
            else
                disp("Error: Function argument only accepts 'Harris' or 'ORB'.");
                return
            end
            [features1, validPoints1] = extractFeatures(roofs1Gray, roofs1Features);
            [features2, validPoints2] = extractFeatures(roofs2Gray, roofs2Features);

            %% Match features of both images
            indexPairs = matchFeatures(features1, features2);
            matchedPoints1 = validPoints1(indexPairs(:,1),:);
            matchedPoints2 = validPoints2(indexPairs(:,2),:);

            %% Display matched features
            figure;
            showMatchedFeatures(roofs1, roofs2, matchedPoints1, matchedPoints2, 'montage');
        end

        function RANSAC(technique)
            %% Read in images
            kfc1 = imread('kfc1(1).jpg');
            kfc2 = imread('kfc2(1).jpg');

            %% Make grayscale copies
            kfc1Gray = rgb2gray(kfc1);
            kfc2Gray = rgb2gray(kfc2);

            %% Extract features of the images
            if strcmp(technique, 'Harris')
                kfc1Features = detectHarrisFeatures(kfc1Gray);
                kfc2Features = detectHarrisFeatures(kfc2Gray);
            elseif strcmp(technique, 'ORB')
                kfc1Features = detectORBFeatures(kfc1Gray);
                kfc2Features = detectORBFeatures(kfc2Gray);
            else
                disp("Error: Function argument only accepts 'Harris' or 'ORB'.");
                return
            end
            [features1, validPoints1] = extractFeatures(kfc1Gray, kfc1Features);
            [features2, validPoints2] = extractFeatures(kfc2Gray, kfc2Features);

            %% Match features of both images
            indexPairs = matchFeatures(features1, features2);
            matchedPoints1 = validPoints1(indexPairs(:,1),:);
            matchedPoints2 = validPoints2(indexPairs(:,2),:);

            %% Estimate geometric transform from image 1 to 2
            transform = estimateGeometricTransform(matchedPoints2, matchedPoints1, 'similarity');
            disp("Angle of image rotation: ")
            disp(180/pi * acos(transform.T(1,1)));
            montage({kfc1, imwarp(kfc2, transform)});
        end

        function detectCheckerboard(realCheckerboard)
            I1 = imread(realCheckerboard);
            I1G = rgb2gray(I1);
            F1 = detectORBFeatures(I1G, 'NumLevels',3);
            imshow(I1);
            hold on;
            scatter(F1.Location(:,1), F1.Location(:,2), 'r*');
            labels1 = dbscan(F1.Location, 45, 10);
            uniqueLabels1 = unique(labels1);
            disp(length(uniqueLabels1));
            uniqueLabels1(uniqueLabels1 == -1) = [];
            meanPoints1 = zeros(length(uniqueLabels1), 2);
            for i = 1:length(uniqueLabels1)
                % Get the points in the current cluster
                clusterPoints1 = F1.Location(labels1 == uniqueLabels1(i), :);

                % Compute the mean of the cluster points
                meanPoints1(i, :) = mean(clusterPoints1, 1);
            end
            scatter(meanPoints1(:,1), meanPoints1(:,2), 'go');


            I2 = imread('modelCheckerboard.png');
            I2G = rgb2gray(I2);
            pos = [100,150,2000,1200];
            F2 = detectORBFeatures(I2G, 'NumLevels',3, 'ROI',pos);
            
            % hold on;
            % disp(F2.Count);
            % imshow(I2);
            % rectangle('Position',pos);

            % scatter(F2.Location(:,1), F2.Location(:,2), 'r*');
            labels2 = dbscan(F2.Location, 45, 2);
            uniqueLabels2 = unique(labels2);
            disp(length(uniqueLabels2));
            uniqueLabels2(uniqueLabels2 == -1) = [];
            meanPoints2 = zeros(length(uniqueLabels2), 2);
            for i = 1:length(uniqueLabels2)
                % Get the points in the current cluster
                clusterPoints2 = F2.Location(labels2 == uniqueLabels2(i), :);

                % Compute the mean of the cluster points
                meanPoints2(i, :) = mean(clusterPoints2, 1);
            end
            % scatter(meanPoints(:,1), meanPoints(:,2), 'go');
            % disp(meanPoints)

            [features1, validPoints1] = extractFeatures(I1G, meanPoints1);
            [features2, validPoints2] = extractFeatures(I2G, meanPoints2);
            indexPairs = matchFeatures(features1, features2, "MatchThreshold",0.00000001);
            disp(size(features1));
            disp(size(features2));
            disp(size(indexPairs));
        end
    end
end