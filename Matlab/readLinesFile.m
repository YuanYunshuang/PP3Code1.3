function [ lines ] = readLinesFile( path )
fid = fopen(path);
lines = struct([]);
while ~feof(fid)
    tline = fgetl(fid);
    line.name = tline;
    lines_matrix = zeros(3,3);
     
    tline = fgetl(fid);
    lines_matrix(1,1) = str2num(tline(1:8));
    lines_matrix(1,2) = str2num(tline(10:17));
    lines_matrix(1,3) = str2num(tline(19:end));
    tline = fgetl(fid);
    lines_matrix(2,1) = str2num(tline(1:8));
    lines_matrix(2,2) = str2num(tline(10:17));
    lines_matrix(2,3) = str2num(tline(19:end));
    tline = fgetl(fid);
    lines_matrix(3,1) = str2num(tline(1:8));
    lines_matrix(3,2) = str2num(tline(10:17));
    lines_matrix(3,3) = str2num(tline(19:end));
    
    line.data = lines_matrix;
    lines = [lines line];
end


end

