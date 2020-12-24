function detectionClusters = clusterDetections(detectinos, vehicleSize)
N = numel(detectinos);
distances = zeros(N);
for i = 1:N
  for j = i+1:N
    if detectinos{i}.SensorIndex == detectinos{j}.SensorIndex
      distances(i,j) = norm(detectinos{i}.Measurement(1:2) - detectinos{j}.Measurement(1:2));
    else
      distances(i,j) = inf;
    end
end
leftToCheck = 1:N;
i = 0;
detectionClusters = cell(N,1);

while ~isempty(leftToCheck)
  underConsideration = leftToCheck(1);
  clusterInds = (distances(underConsideration,leftToCheck) < vehicleSize);
  detInds = leftToCheck(clusterInds);
  clusterDets = [detectinos{detInds}];
  clusterMeas = [clusterDets.Measurement];
  meas = mean(clusterMeas,2);
  meas2D = [meas(1:2);meas(4:5)];
  i = i + 1;
  detectionClusters{i} = detectinos{detInds(1)};
  detectionClusters{i}.Measurement = meas2D;
  leftToCheck(clusterInds) = [];
end
detectionClusters(i+1:end) = [];
end