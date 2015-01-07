% extracts camera data for robot i from vector of all camera data
%
% Modified: 2/11/2014 - R. Beard
%
function out = extract_camera_data(in,i)
    out = in(1+(2+4*P.num_robots+2*P.num_markers)*(i-1):(2+4*P.num_robots+2*P.num_markers)*i);
end
  