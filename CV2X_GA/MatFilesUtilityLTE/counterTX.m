function matrix = counterTX(IDvehicleTX,indexVehicleTX,awarenessID,errorMatrix)
% Count correctly transmitted beacons among neighbors within Raw
% Matrix = [#Correctly transmitted beacons, #Errors, #Neighbors]

Ntx = length(indexVehicleTX);
matrix = zeros(Ntx,3);
for i = 1:Ntx

    % #Neighbors of TX vehicle IDvehicleTX(i)
    Nneighbors = nnz(awarenessID(indexVehicleTX(i),:));   %找到比较近的车辆编号
    matrix(i,3) = Nneighbors;

    % #Neighbors that do not have correctly received the beacon
    Nerrors = nnz(errorMatrix(:,1)==IDvehicleTX(i));
    matrix(i,2) = Nerrors;

    % #Neighbors that have correctly received the beacon transmitted by IDvehicleTX(i)
    matrix(i,1) = Nneighbors - Nerrors;

end

