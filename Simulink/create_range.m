function timetbl = create_range(startvalue, endvalue, ndivisions, rate)
range = zeros(1, ndivisions);
index = 1;

if startvalue > endvalue
    index = ndivisions;
    for i = endvalue:(startvalue-endvalue)/(ndivisions-1):startvalue
       range(index) = i;
       index = index-1;
    end
elseif startvalue < endvalue
    for i = startvalue:(endvalue-startvalue)/(ndivisions-1):endvalue
       range(index) = i;
       index = index+1;
    end
    
else %startvalue = endvalue
    for i = 1:ndivisions-1
       range(i) = startvalue; 
    end
end

timetbl = timetable(range.', 'SampleRate', 10)
end