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
                features = detectORBFeatures(imageGray, 'NumLevels',1, 'ScaleFactor', 1.1);
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
            labels = dbscan(features.Location, 5, 4);

            currentPoint = 1;
            for i = 1:features.Count
                if labels(i) == currentPoint
                    scatter(features.Location(i,1), features.Location(i,2), 'go');
                    currentPoint = currentPoint + 1;
                end
                % if currentPoint > 28
                %     break;
                % end
            end
            disp(currentPoint - 1);
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
    end
end